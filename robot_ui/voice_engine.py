import asyncio
import json
import os
import queue
import re
import subprocess
import tempfile
import threading
import time
from contextlib import contextmanager

import edge_tts
import speech_recognition as sr
from PyQt6.QtCore import QObject, pyqtSignal

EDGE_TTS_VOICE  = "vi-VN-HoaiMyNeural"
EDGE_TTS_RATE   = "+30%"
ESPEAK_LANG     = "vi-vn-x-central"
ESPEAK_RATE     = 150
MIC_DEVICE_NAME = "USB2.0 Device"
STT_LANGUAGE    = "vi-VN"

WAKE_TRIGGERS = ["ê mày", "e mày", "e may", "ê dách con", "e dách con", "start", "hey zackon", "alo alo"]

UI_COMMANDS = {
    "tải bản đồ":  "LOAD_MAP",
    "tải map":     "LOAD_MAP",
    "chọn bản đồ": "LOAD_MAP",
    "chọn map":    "LOAD_MAP",
    "tạo map":     "NEW_MAP",
    "tạo bản đồ":  "NEW_MAP",
}

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
    IDLE      = "[--] IDLE"
    LISTENING = "[>>] LISTENING"
    THINKING  = "[..] THINKING"
    SPEAKING  = "[<<] SPEAKING"


def _find_mic_index(name: str | None) -> int | None:
    if name is None:
        return None
    return next(
        (i for i, n in enumerate(sr.Microphone.list_microphone_names()) if name.lower() in n.lower()),
        None
    )


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


def _fetch_edge_audio(text: str, voice: str, rate: str) -> bytes:
    async def _collect():
        chunks = []
        async for chunk in edge_tts.Communicate(text, voice, rate=rate).stream():
            if chunk["type"] == "audio":
                chunks.append(chunk["data"])
        return b''.join(chunks)
    try:
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(_collect())
        finally:
            loop.close()
    except Exception:
        return b''


class VoiceEngine(QObject):
    state_changed    = pyqtSignal(str)
    transcript_ready = pyqtSignal(str)
    waypoint_command = pyqtSignal(str)
    ui_command       = pyqtSignal(str)

    def __init__(self, edge_voice: str | None = EDGE_TTS_VOICE):
        super().__init__()
        self._edge_voice  = edge_voice
        self._running     = False
        self._state_lock  = threading.Lock()
        self._play_lock   = threading.Lock()
        self._stop_flag   = threading.Event()

        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 400
        self.recognizer.pause_threshold  = 1.5

        self._tts_queue = queue.Queue()
        self._state     = VoiceState.IDLE
        threading.Thread(target=self._tts_worker, daemon=True).start()

    def set_voice(self, edge_voice: str) -> None:
        self._edge_voice = edge_voice

    def start(self):
        with self._state_lock:
            if self._running:
                return
            self._running = True
        self._set_state(VoiceState.IDLE)
        threading.Thread(target=self._listen_loop, daemon=True).start()

    def stop(self):
        with self._state_lock:
            self._running = False
        self._set_state(VoiceState.IDLE)

    def stop_speaking(self):
        self._stop_flag.set()
        while not self._tts_queue.empty():
            try:
                self._tts_queue.get_nowait()
                self._tts_queue.task_done()
            except queue.Empty:
                break
        if self._running:
            self._set_state(VoiceState.IDLE)

    def speak(self, text: str):
        text = re.sub(r'<[^>]+>', '', text).strip()  # strip HTML/action tags
        if text:
            self._tts_queue.put(text)

    def _set_state(self, state: str):
        self._state = state
        self.state_changed.emit(state)

    def _tts_worker(self):
        while True:
            text = self._tts_queue.get()
            self._stop_flag.clear()
            self._set_state(VoiceState.SPEAKING)
            if self._edge_voice:
                audio = _fetch_edge_audio(text, self._edge_voice, EDGE_TTS_RATE)
                if audio and not self._stop_flag.is_set():
                    self._play_mp3(audio)
                elif not audio:
                    self._speak_espeak(text)
            else:
                self._speak_espeak(text)
            self._tts_queue.task_done()
            if self._tts_queue.empty() and self._running:
                self._set_state(VoiceState.IDLE)

    def _play_mp3(self, audio: bytes):
        with self._play_lock:
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
                f.write(audio)
                tmp_path = f.name
            try:
                proc = subprocess.Popen(
                    ['mpg123', '-q', tmp_path],
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

    def _speak_espeak(self, text: str):
        try:
            subprocess.run(
                ['espeak-ng', '-v', ESPEAK_LANG, '-s', str(ESPEAK_RATE), '-a', '200', '--', text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                check=True,
            )
        except FileNotFoundError:
            print("[VoiceEngine] espeak-ng not found — sudo apt install espeak-ng")
        except Exception as e:
            print(f"[VoiceEngine] espeak error: {e}")

    def _listen_loop(self):
        mic_index = _find_mic_index(MIC_DEVICE_NAME)
        with _suppress_stderr():
            mic = sr.Microphone(device_index=mic_index)
            source = mic.__enter__()
        try:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            while self._running:
                self._process_wake_word(source)
        finally:
            mic.__exit__(None, None, None)

    def _process_wake_word(self, source):
        try:
            audio = self.recognizer.listen(source, timeout=2.0, phrase_time_limit=3.0)
            text  = self.recognizer.recognize_google(audio, language=STT_LANGUAGE).lower()
            if text in UI_COMMANDS:
                self.ui_command.emit(UI_COMMANDS[text])
            elif self._check_waypoint_command(text) is not None:
                self.waypoint_command.emit(self._check_waypoint_command(text))
            elif any(w in text for w in WAKE_TRIGGERS):
                self._handle_command(source)
        except (sr.WaitTimeoutError, sr.UnknownValueError):
            pass
        except sr.RequestError as e:
            print(f"[VoiceEngine] STT error: {e}")
            time.sleep(0.5)
        except Exception as e:
            print(f"[VoiceEngine] Listen error: {e}")
            time.sleep(0.5)

    def _handle_command(self, source):
        self._set_state(VoiceState.LISTENING)
        try:
            audio = self.recognizer.listen(source, timeout=10.0, phrase_time_limit=10.0)
        except sr.WaitTimeoutError:
            self._set_state(VoiceState.IDLE)
            return
        self._set_state(VoiceState.THINKING)
        try:
            text = self.recognizer.recognize_google(audio, language=STT_LANGUAGE)
            lower = text.lower()
            if lower in UI_COMMANDS:
                self.ui_command.emit(UI_COMMANDS[lower])
                self._set_state(VoiceState.IDLE)
                return
            slot = self._check_waypoint_command(lower)
            if slot is not None:
                self.waypoint_command.emit(slot)
                self._set_state(VoiceState.IDLE)
            else:
                self.transcript_ready.emit(text)
        except (sr.UnknownValueError, sr.WaitTimeoutError):
            self._set_state(VoiceState.IDLE)
        except Exception as e:
            print(f"[VoiceEngine] Command error: {e}")
            self._set_state(VoiceState.IDLE)

    def _load_waypoint_keys(self) -> list[str]:
        try:
            wp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints.json')
            with open(wp_file, 'r', encoding='utf-8') as f:
                return list(json.load(f).keys())
        except Exception:
            return []

    def _check_waypoint_command(self, text: str) -> str | None:
        text = text.strip()
        keys = self._load_waypoint_keys()
        for prefix in WAYPOINT_PREFIXES:
            if not text.startswith(prefix):
                continue
            remainder = text[len(prefix):].strip()
            if remainder.isdigit():
                return remainder
            if remainder in VI_DIGITS:
                return VI_DIGITS[remainder]
            matched = next((k for k in keys if remainder == k.lower()), None)
            if matched:
                return matched
        return next((k for k in keys if text == k.lower()), None)
