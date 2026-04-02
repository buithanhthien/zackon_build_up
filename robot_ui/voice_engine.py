import asyncio
import json
import os
import queue
import re
import subprocess
import threading
import time
from contextlib import contextmanager

import edge_tts
import speech_recognition as sr
from PyQt6.QtCore import QObject, pyqtSignal

EDGE_TTS_VOICE = "vi-VN-HoaiMyNeural"
MIC_DEVICE_NAME = "USB2.0 Device"

# Put your recorded wake word samples in this folder (3-4 .wav files).
# Record with: lwake record wake_refs/sample-1.wav
WAKE_TRIGGERS = ["ê mày", "e mày", "e may", "ê dách con", "e dách con", "start", "hey zackon", "alo alo"]


def _find_mic_index(name: str | None) -> int | None:
    if name is None:
        return None
    return next(
        (i for i, n in enumerate(sr.Microphone.list_microphone_names()) if name.lower() in n.lower()),
        None
    )


@contextmanager
def suppress_stderr():
    null_fd = os.open(os.devnull, os.O_RDWR)
    save_fd = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        os.dup2(save_fd, 2)
        os.close(null_fd)
        os.close(save_fd)


class VoiceState:
    IDLE      = "[--] IDLE"
    LISTENING = "[>>] LISTENING"
    THINKING  = "[..] THINKING"
    SPEAKING  = "[<<] SPEAKING"


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


class VoiceEngine(QObject):
    state_changed    = pyqtSignal(str)
    transcript_ready = pyqtSignal(str)
    waypoint_command = pyqtSignal(str)

    def __init__(self, edge_voice: str | None = EDGE_TTS_VOICE):
        super().__init__()
        self._edge_voice  = edge_voice
        self._edge_rate   = "+30%"
        self._espeak_lang = "vi-vn-x-central"
        self._espeak_rate = 150

        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 400
        self.recognizer.pause_threshold  = 0.7

        self._running       = False
        self._state_lock    = threading.Lock()
        self._current_state = VoiceState.IDLE

        self._tts_queue  = queue.Queue()
        self._tts_thread = threading.Thread(target=self._tts_worker, daemon=True)
        self._tts_thread.start()

        print(f"[VoiceEngine] Active voice: {self._edge_voice or 'espeak-ng/' + self._espeak_lang}")

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

    def speak(self, text: str):
        clean = re.sub(r'<[^>]+>', '', text).strip()
        if clean:
            self._tts_queue.put(clean)

    def _tts_worker(self):
        proc = subprocess.Popen(
            ['mpg123', '-q', '-'],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        while True:
            text = self._tts_queue.get()
            self._set_state(VoiceState.SPEAKING)
            if self._edge_voice:
                audio = self._fetch_edge_audio(text)
                if audio:
                    try:
                        proc.stdin.write(audio)
                        proc.stdin.flush()
                    except Exception:
                        proc = subprocess.Popen(
                            ['mpg123', '-q', '-'],
                            stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )
                        proc.stdin.write(audio)
                        proc.stdin.flush()
                else:
                    self._speak_espeak(text)
            else:
                self._speak_espeak(text)
            self._tts_queue.task_done()
            if self._tts_queue.empty() and self._running:
                self._set_state(VoiceState.IDLE)

    def _fetch_edge_audio(self, text: str) -> bytes:
        try:
            communicate = edge_tts.Communicate(text, self._edge_voice, rate=self._edge_rate)
            chunks = []
            async def _collect():
                async for chunk in communicate.stream():
                    if chunk["type"] == "audio":
                        chunks.append(chunk["data"])
            asyncio.run(_collect())
            return b''.join(chunks)
        except Exception:
            return b''

    def _speak_espeak(self, text: str):
        try:
            subprocess.run(
                ['espeak-ng', '-v', self._espeak_lang,
                 '-s', str(self._espeak_rate), '-a', '200', '--', text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                check=True)
        except FileNotFoundError:
            print("[VoiceEngine] espeak-ng not found — sudo apt install espeak-ng")
        except Exception as e:
            print(f"[VoiceEngine] TTS error: {e}")
        finally:
            if self._running:
                self._set_state(VoiceState.IDLE)

    def _set_state(self, state: str):
        self._current_state = state
        self.state_changed.emit(state)

    def _listen_loop(self):
        mic_index = _find_mic_index(MIC_DEVICE_NAME)
        with suppress_stderr():
            mic = sr.Microphone(device_index=mic_index)
            source = mic.__enter__()
        try:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            print("[VoiceEngine] Listening for wake word...")
            while self._running:
                if self._current_state != VoiceState.IDLE:
                    time.sleep(0.05)
                    continue
                try:
                    audio = self.recognizer.listen(source, timeout=2.0, phrase_time_limit=3.0)
                    text = self.recognizer.recognize_google(audio, language='vi-VN').lower()
                    print(f"[VoiceEngine] Heard: '{text}'")
                    slot = self._check_waypoint_command(text)
                    if slot is not None:
                        self.waypoint_command.emit(slot)
                    elif any(w in text for w in WAKE_TRIGGERS):
                        print("[VoiceEngine] Wake word detected!")
                        self._handle_command(source)
                except (sr.WaitTimeoutError, sr.UnknownValueError):
                    pass
                except sr.RequestError as e:
                    print(f"[VoiceEngine] STT error: {e}")
                    time.sleep(1)
                except Exception as e:
                    print(f"[VoiceEngine] Listen error: {e}")
                    time.sleep(1)
        finally:
            mic.__exit__(None, None, None)

    def _handle_command(self, source):
        self._set_state(VoiceState.LISTENING)
        try:
            audio = self.recognizer.listen(source, timeout=4.0, phrase_time_limit=10.0)
        except sr.WaitTimeoutError:
            self._set_state(VoiceState.IDLE)
            return
        self._set_state(VoiceState.THINKING)
        try:
            text = self.recognizer.recognize_google(audio, language='vi-VN')
            print(f"[VoiceEngine] Command: '{text}'")
            slot = self._check_waypoint_command(text.lower())
            if slot is not None:
                self.waypoint_command.emit(slot)
                self._set_state(VoiceState.IDLE)
            else:
                self.transcript_ready.emit(text)
        except (sr.UnknownValueError, sr.WaitTimeoutError):
            self._set_state(VoiceState.IDLE)
        except Exception as e:
            print(f"[VoiceEngine] Command STT error: {e}")
            self._set_state(VoiceState.IDLE)

    def _load_waypoint_keys(self):
        try:
            wp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'waypoints.json')
            with open(wp_file, 'r', encoding='utf-8') as f:
                return list(json.load(f).keys())
        except Exception:
            return []

    def _check_waypoint_command(self, text: str) -> str | None:
        text = text.strip()
        for prefix in WAYPOINT_PREFIXES:
            if text.startswith(prefix):
                remainder = text[len(prefix):].strip()
                if remainder.isdigit():
                    return remainder
                if remainder in VI_DIGITS:
                    return VI_DIGITS[remainder]
                for key in self._load_waypoint_keys():
                    if remainder == key.lower():
                        return key
        for key in self._load_waypoint_keys():
            if text == key.lower():
                return key
        return None
