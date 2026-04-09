import json
import os
import queue
import re
import subprocess
import tempfile
import threading
import time
from contextlib import contextmanager

import speech_recognition as sr
from PyQt6.QtCore import QObject, pyqtSignal
from vieneu import Vieneu

MIC_DEVICE_PRIORITY = [
    "pipewire",       # PipeWire virtual device (modern Linux audio server)
    "sysdefault",     # ALSA system default device
    "USB2.0 Device",  # Generic USB microphone/webcam mic
    "SN6140 Analog",  # Realtek onboard analog mic (common on mini PCs)
    "DMIC16kHz",      # Digital mic array at 16kHz (Intel/AMD laptops)
    "DMIC",           # Generic digital mic array
]
MIC_SAMPLE_RATE = 16000
STT_LANGUAGE    = "vi-VN"

WAYPOINT_PREFIXES = [
    "đi tới vị trí số", "đi đến vị trí số", "di tới vị trí số", "di đến vị trí số",
    "đi tới vị trí", "đi đến vị trí", "di tới vị trí", "di đến vị trí",
    "đi tới số", "đi đến số", "di tới số", "di đến số",
    "tới vị trí số", "đến vị trí số", "tới vị trí", "đến vị trí",
    "tới số", "đến số",
    "đi tới", "đi đến", "di tới", "di đến",
]
VI_DIGITS = {
    "một": "1", "hai": "2", "ba": "3", "bốn": "4", "năm": "5",
    "sáu": "6", "bảy": "7", "tám": "8", "chín": "9", "mười": "10",
}


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
    waypoint_command = pyqtSignal(str)
    ui_command       = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._play_lock  = threading.Lock()
        self._stop_flag  = threading.Event()

        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
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
        threading.Thread(target=self._listen_thread, daemon=True).start()

    def _listen_thread(self):
        available = sr.Microphone.list_microphone_names()
        candidates = []
        for name in MIC_DEVICE_PRIORITY:
            idx = next((i for i, n in enumerate(available) if name.lower() in n.lower()), None)
            if idx is not None:
                candidates.append(idx)
        candidates.append(None)  # system default as final fallback

        source = None
        mic = None
        for idx in candidates:
            try:
                mic = sr.Microphone(device_index=idx, sample_rate=MIC_SAMPLE_RATE)
                with _suppress_stderr():
                    source = mic.__enter__()
                print(f"[VoiceEngine] using mic index={idx} ({available[idx] if idx is not None else 'default'})")
                break
            except Exception as e:
                print(f"[VoiceEngine] mic index={idx} failed: {e}")
                source = None

        if source is None:
            print("[VoiceEngine] no usable microphone found")
            self.state_changed.emit("")
            return
        try:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            print(f"[VoiceEngine] energy_threshold={self.recognizer.energy_threshold:.1f}, listening...")
            self._set_state(VoiceState.LISTENING)
            try:
                audio = self.recognizer.listen(source, timeout=10.0, phrase_time_limit=20.0)
                print(f"[VoiceEngine] audio captured, sending to Google STT...")
            except sr.WaitTimeoutError:
                print(f"[VoiceEngine] timeout — no speech detected")
                self.state_changed.emit("")
                return
            self._set_state(VoiceState.THINKING)
            try:
                text = self.recognizer.recognize_google(audio, language=STT_LANGUAGE)
                print(f"[VoiceEngine] recognized: '{text}'")
                slot = self._check_waypoint_command(text.lower())
                if slot is not None:
                    self.waypoint_command.emit(slot)
                else:
                    self.transcript_ready.emit(text)
            except sr.UnknownValueError:
                print(f"[VoiceEngine] could not understand audio")
            except Exception as e:
                print(f"[VoiceEngine] STT error: {e}")
        finally:
            self.state_changed.emit("")
            try:
                mic.__exit__(None, None, None)
            except Exception:
                pass

    def stop_speaking(self):
        self._stop_flag.set()
        try:
            while True:
                self._tts_queue.get_nowait()
                self._tts_queue.task_done()
        except queue.Empty:
            pass

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

    def _load_waypoints(self) -> dict:
        try:
            wp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints.json')
            with open(wp_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return {}

    def _check_waypoint_command(self, text: str) -> str | None:
        text = text.strip()
        wps = self._load_waypoints()
        keys = list(wps.keys())

        def _norm(s: str) -> str:
            return re.sub(r'([a-z])(\d+)[.,](\d+)', r'\1\2\3', s.lower())

        def _matches(remainder: str, key: str) -> bool:
            r = _norm(remainder)
            if r == _norm(key):
                return True
            return any(r == _norm(a) for a in wps[key].get("aliases", []))

        for prefix in WAYPOINT_PREFIXES:
            if not text.startswith(prefix):
                continue
            remainder = text[len(prefix):].strip()
            if remainder.isdigit():
                return remainder
            if remainder in VI_DIGITS:
                return VI_DIGITS[remainder]
            matched = next((k for k in keys if _matches(remainder, k)), None)
            if matched:
                return matched
        return None
