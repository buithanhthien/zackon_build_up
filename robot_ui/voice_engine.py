import os
import queue
import re
import subprocess
import tempfile
import threading
import time
from contextlib import contextmanager

_env_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), '.env')
if os.path.exists(_env_path):
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith('#') and '=' in _line:
                _k, _v = _line.split('=', 1)
                os.environ.setdefault(_k.strip(), _v.strip())

import speech_recognition as sr
from PyQt6.QtCore import QObject, pyqtSignal, pyqtSlot
from vieneu import Vieneu

MIC_DEVICE_PRIORITY = [
    "pipewire",       # PipeWire routes all physical mics (USB, 3.5mm jack)
    "sysdefault",
    "default",
]
MIC_JACK_KEYWORDS = ["sn6140", "analog"]  # 3.5mm jack identifiers (exclude HDMI)
MIC_JACK_EXCLUDE  = ["hdmi", "iec958", "spdif"]  # never treat these as mic input
MIC_FORCE_INDEX = None
MIC_SAMPLE_RATE = 16000
STT_LANGUAGE    = "vi-VN"


class VoiceState:
    LISTENING = "[>>] LISTENING"
    THINKING  = "[..] THINKING"
    SPEAKING  = "[<<] SPEAKING"


@contextmanager
def _suppress_stderr():
    null_fd = os.open(os.devnull, os.O_RDWR)
    save_fd = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        os.dup2(save_fd, 2)
        os.close(null_fd)
        os.close(save_fd)



class VoiceEngine(QObject):
    state_changed    = pyqtSignal(str)
    transcript_ready = pyqtSignal(str)
    ui_command       = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._play_lock    = threading.Lock()
        self._stop_flag    = threading.Event()
        self._listen_lock  = threading.Lock()

        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.energy_threshold = 500
        self.recognizer.pause_threshold  = 1.5

        self._tts_queue = queue.Queue()
        self._tts_engine = None
        threading.Thread(target=self._tts_worker, daemon=True).start()

    def _init_tts(self):
        if self._tts_engine is None:
            print("[TTS] Initializing VieNeu-TTS...")
            self._tts_engine = Vieneu()
            print("[TTS] VieNeu-TTS ready")

    def listen_once(self):
        if not self._listen_lock.acquire(blocking=False):
            print("[VoiceEngine] already listening, ignoring duplicate request")
            return
        threading.Thread(target=self._listen_thread, daemon=True).start()

    def _listen_thread(self):
        available = sr.Microphone.list_microphone_names()

        # Log jack mic availability
        jack_idx = next((i for i, n in enumerate(available)
                         if any(k in n.lower() for k in MIC_JACK_KEYWORDS)
                         and not any(x in n.lower() for x in MIC_JACK_EXCLUDE)), None)
        if jack_idx is not None:
            print(f"[VoiceEngine] 3.5mm jack mic detected: index={jack_idx} ({available[jack_idx]})")
        else:
            print("[VoiceEngine] 3.5mm jack mic NOT detected")
        if MIC_FORCE_INDEX is not None:
            candidates = [MIC_FORCE_INDEX, None]
        else:
            candidates = []
            # Prioritize 3.5mm jack if detected
            if jack_idx is not None:
                candidates.append(jack_idx)
            # Then try priority list
            for name in MIC_DEVICE_PRIORITY:
                idx = next((i for i, n in enumerate(available) if name.lower() in n.lower()), None)
                if idx is not None and idx not in candidates:
                    candidates.append(idx)
            candidates.append(None)  # system default as final fallback

        mic = None
        chosen_idx = None
        for idx in candidates:
            try:
                candidate = sr.Microphone(device_index=idx, sample_rate=MIC_SAMPLE_RATE)
                # Open the stream first to verify the device is usable
                candidate.__enter__()
                try:
                    with _suppress_stderr():
                        self.recognizer.adjust_for_ambient_noise(candidate, duration=0.5)
                    mic = candidate
                    chosen_idx = idx
                    print(f"[VoiceEngine] using mic index={idx} ({available[idx] if idx is not None else 'default'})")
                    break
                except Exception as e:
                    print(f"[VoiceEngine] mic index={idx} failed: {e}")
                finally:
                    try:
                        candidate.__exit__(None, None, None)
                    except Exception:
                        pass
            except Exception as e:
                print(f"[VoiceEngine] mic index={idx} open failed: {e}")

        if mic is None:
            print("[VoiceEngine] no usable microphone found")
            self.state_changed.emit("")
            self._listen_lock.release()
            return

        try:
            with mic as source:
                print(f"[VoiceEngine] energy_threshold={self.recognizer.energy_threshold:.1f}, listening...")
                self._set_state(VoiceState.LISTENING)
                try:
                    audio = self.recognizer.listen(source, timeout=10.0, phrase_time_limit=15.0)
                    print(f"[VoiceEngine] audio captured, sending to Google STT...")
                except sr.WaitTimeoutError:
                    print(f"[VoiceEngine] timeout — no speech detected")
                    return
            self._set_state(VoiceState.THINKING)
            try:
                text = self.recognizer.recognize_google(audio, language=STT_LANGUAGE)
                print(f"[VoiceEngine] recognized: '{text}'")
                self.transcript_ready.emit(text)
            except sr.UnknownValueError:
                print(f"[VoiceEngine] could not understand audio")
            except Exception as e:
                print(f"[VoiceEngine] STT error: {e}")
        finally:
            self.state_changed.emit("")
            self._listen_lock.release()

    def stop_speaking(self):
        self._stop_flag.set()
        try:
            while True:
                self._tts_queue.get_nowait()
                self._tts_queue.task_done()
        except queue.Empty:
            pass

    @pyqtSlot(str)
    def speak(self, text: str):
        text = re.sub(r'<[^>]+>', '', text)
        text = re.sub(r'\*\*([^*]+)\*\*', r'\1', text)
        text = re.sub(r'\*([^*]+)\*', r'\1', text)
        text = text.replace('"', '').replace('"', '').replace('"', '')
        text = re.sub(r'\.(?=[a-zA-Z])', ' chấm ', text)
        text = text.strip()
        if text:
            self._tts_queue.put(text)

    def _set_state(self, state: str):
        self.state_changed.emit(state)

    def _tts_worker(self):
        while True:
            text = self._tts_queue.get()
            self._stop_flag.clear()
            
            if self._tts_engine is None:
                self._init_tts()
            
            try:
                audio = self._tts_engine.infer(text=text)
                if audio is not None and len(audio) > 0 and not self._stop_flag.is_set():
                    self._set_state(VoiceState.SPEAKING)
                    self._play_wav(audio)
            except Exception as e:
                print(f"[TTS] synthesis failed: {e}")
            
            self._tts_queue.task_done()
            if self._tts_queue.empty():
                self._set_state("")

    def _play_wav(self, audio):
        with self._play_lock:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                self._tts_engine.save(audio, f.name)
                tmp_path = f.name
            try:
                proc = subprocess.Popen(
                    ['aplay', '-q', tmp_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                while proc.poll() is None:
                    if self._stop_flag.is_set():
                        proc.kill()
                        break
                    time.sleep(0.05)
            finally:
                os.unlink(tmp_path)
