import threading
import time
import re
import os
import asyncio
import subprocess
import tempfile
from contextlib import contextmanager
import speech_recognition as sr
import edge_tts
from PyQt6.QtCore import QObject, pyqtSignal

# ── TTS voice ─────────────────────────────────────────────────────────────────
#   vi-VN-HoaiMyNeural  – Vietnamese female  ← current
#   vi-VN-NamMinhNeural – Vietnamese male
# Set to None to fall back to offline espeak-ng.
EDGE_TTS_VOICE = "vi-VN-HoaiMyNeural"

# ── Wake word ─────────────────────────────────────────────────────────────────
# Aliases cover common Google STT transcriptions of "Ê mày".
WAKE_WORD = "sủa tao nghe"
WAKE_ALIASES = [WAKE_WORD, "sửa tao nghe", 
                "sủa tào nghe", 
                "sủa ta nghe", 
                "sủa tao nghen", 
                "ủa tao nghe"]

# ─────────────────────────────────────────────────────────────────────────────


@contextmanager
def suppress_stderr():
    """Redirect C-level stderr to /dev/null (suppresses ALSA noise)."""
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
    IDLE      = "🔴 IDLE"
    LISTENING = "🟢 LISTENING"
    THINKING  = "⏳ THINKING"
    SPEAKING  = "🔊 SPEAKING"


class VoiceEngine(QObject):
    state_changed    = pyqtSignal(str)   # emits VoiceState constants
    transcript_ready = pyqtSignal(str)   # emits transcribed user text

    def __init__(self, wake_word: str = WAKE_WORD,
                 edge_voice: str | None = EDGE_TTS_VOICE):
        super().__init__()
        self.wake_word   = wake_word.lower()
        self._edge_voice = edge_voice          # e.g. "vi-VN-HoaiMyNeural"
        self._espeak_lang = "vi-vn-x-central"  # offline fallback lang
        self._espeak_rate = 160                # words per minute

        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 400
        self.recognizer.pause_threshold  = 0.8

        print(f"[VoiceEngine] Active voice: "
              f"{self._edge_voice or 'espeak-ng/' + self._espeak_lang}")

        self._running          = False
        self._listening_thread = None
        self._state_lock       = threading.Lock()
        self._current_state    = VoiceState.IDLE

        with suppress_stderr():
            self._microphone = sr.Microphone()

    # ── Public API ────────────────────────────────────────────────────────────

    def set_voice(self, edge_voice: str) -> None:
        """Switch the edge-tts neural voice at runtime.
        e.g. 'vi-VN-HoaiMyNeural' or 'vi-VN-NamMinhNeural'."""
        self._edge_voice = edge_voice
        print(f"[VoiceEngine] Voice changed to: {edge_voice}")

    def start(self):
        with self._state_lock:
            if self._running:
                return
            self._running = True
        self._set_state(VoiceState.IDLE)
        self._listening_thread = threading.Thread(
            target=self._listen_loop, daemon=True)
        self._listening_thread.start()

    def stop(self):
        with self._state_lock:
            self._running = False
            self._set_state(VoiceState.IDLE)

    def speak(self, text: str):
        """Speak AI response text (strips action tags, runs async)."""
        clean = re.sub(r'<[^>]+>', '', text).strip()
        if not clean:
            return
        self._set_state(VoiceState.SPEAKING)
        threading.Thread(target=self._speak_blocking,
                         args=(clean,), daemon=True).start()

    # ── TTS ───────────────────────────────────────────────────────────────────

    def _speak_blocking(self, text: str):
        if self._edge_voice:
            self._speak_edge(text)
        else:
            self._speak_espeak(text)

    def _speak_edge(self, text: str):
        """Primary TTS: Microsoft Edge neural voice via edge-tts + mpg123."""
        tmp = None
        try:
            tmp = tempfile.NamedTemporaryFile(suffix='.mp3', delete=False)
            tmp.close()
            asyncio.run(edge_tts.Communicate(text, self._edge_voice).save(tmp.name))
            subprocess.run(['mpg123', '-q', tmp.name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           check=True)
        except FileNotFoundError:
            print("[VoiceEngine] mpg123 not found — sudo apt install mpg123")
        except Exception as e:
            print(f"[VoiceEngine] edge-tts error: {e}  →  falling back to espeak-ng")
            self._speak_espeak(text)
        finally:
            if tmp and os.path.exists(tmp.name):
                os.remove(tmp.name)
            if self._running:
                self._set_state(VoiceState.IDLE)

    def _speak_espeak(self, text: str):
        """Offline fallback TTS: espeak-ng subprocess."""
        try:
            subprocess.run(
                ['espeak-ng', '-v', self._espeak_lang,
                 '-s', str(self._espeak_rate), '-a', '200', '--', text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                check=True)
        except FileNotFoundError:
            print("[VoiceEngine] espeak-ng not found — sudo apt install espeak-ng")
        except subprocess.CalledProcessError as e:
            print(f"[VoiceEngine] espeak-ng error (exit {e.returncode})")
        except Exception as e:
            print(f"[VoiceEngine] TTS error: {e}")
        finally:
            if self._running:
                self._set_state(VoiceState.IDLE)

    # ── STT / listen loop ─────────────────────────────────────────────────────

    def _set_state(self, state: str):
        self._current_state = state
        self.state_changed.emit(state)

    def _listen_loop(self):
        with suppress_stderr():
            mic = sr.Microphone()
            source = mic.__enter__()
        try:
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            while self._running:
                if self._current_state != VoiceState.IDLE:
                    time.sleep(0.1)
                    continue
                try:
                    audio = self.recognizer.listen(
                        source, timeout=2.0, phrase_time_limit=3.0)
                    try:
                        text = self.recognizer.recognize_google(
                            audio, language='vi-VN').lower()
                        print(f"[VoiceEngine] Heard: '{text}'")
                        all_wake = [self.wake_word] + WAKE_ALIASES
                        if any(w in text for w in all_wake):
                            self._handle_wake_word(source)
                    except sr.UnknownValueError:
                        pass
                    except sr.RequestError as e:
                        print(f"[VoiceEngine] STT error: {e}")
                except sr.WaitTimeoutError:
                    pass
                except Exception as e:
                    print(f"[VoiceEngine] Listen error: {e}")
                    time.sleep(1)
        finally:
            mic.__exit__(None, None, None)

    def _handle_wake_word(self, source):
        self._set_state(VoiceState.LISTENING)
        try:
            audio = self.recognizer.listen(
                source, timeout=4.0, phrase_time_limit=10.0)
            self._set_state(VoiceState.THINKING)
            text = self.recognizer.recognize_google(audio, language='vi-VN')
            self.transcript_ready.emit(text)
        except (sr.WaitTimeoutError, sr.UnknownValueError):
            self._set_state(VoiceState.IDLE)
        except Exception as e:
            print(f"[VoiceEngine] Command STT error: {e}")
            self._set_state(VoiceState.IDLE)
