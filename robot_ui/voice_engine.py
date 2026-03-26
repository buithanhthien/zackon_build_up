import threading
import time
import re
import os
import sys
from contextlib import contextmanager
import speech_recognition as sr
import pyttsx3
from PyQt6.QtCore import QObject, pyqtSignal

@contextmanager
def suppress_stderr():
    """Suppress C-level and Python-level stderr output."""
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
    IDLE = "🔴 IDLE"
    LISTENING = "🟢 LISTENING"
    THINKING = "⏳ THINKING"
    SPEAKING = "🔊 SPEAKING"

class VoiceEngine(QObject):
    state_changed = pyqtSignal(str)          # Emits VoiceState constants
    transcript_ready = pyqtSignal(str)       # Emits transcribed user text

    def __init__(self, wake_word="can you hear me"):
        super().__init__()
        self.wake_word = wake_word.lower()
        self.recognizer = sr.Recognizer()
        
        # Configure microphone settings for better ambient noise handling
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 400
        self.recognizer.pause_threshold = 0.8
        
        with suppress_stderr():
            self.tts_engine = pyttsx3.init()
            # Ensure we have a decent English voice
            voices = self.tts_engine.getProperty('voices')
            for voice in voices:
                if 'english' in voice.name.lower() or 'en_US' in voice.id:
                    self.tts_engine.setProperty('voice', voice.id)
                    break
            
            self.tts_engine.setProperty('rate', 160)  # Slightly faster than default
            
            self._running = False
            self._listening_thread = None
            self._microphone = sr.Microphone()
        
        # State lock
        self._state_lock = threading.Lock()
        self._current_state = VoiceState.IDLE

    def start(self):
        with self._state_lock:
            if self._running:
                return
            self._running = True
            
        self._set_state(VoiceState.IDLE)
        self._listening_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._listening_thread.start()

    def stop(self):
        with self._state_lock:
            self._running = False
            self._set_state(VoiceState.IDLE)

    def speak(self, text):
        """Called by the main thread when AI has a response."""
        clean_text = re.sub(r'<[^>]+>', '', text).strip()
        if not clean_text:
            return
            
        self._set_state(VoiceState.SPEAKING)
        threading.Thread(target=self._speak_blocking, args=(clean_text,), daemon=True).start()

    def _speak_blocking(self, text):
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            print(f"[VoiceEngine] TTS Error: {e}")
        finally:
            if self._running:
                self._set_state(VoiceState.IDLE)

    def _set_state(self, state):
        self._current_state = state
        self.state_changed.emit(state)

    def _listen_loop(self):
        # Open the microphone ONCE to avoid ALSA spam and context manager bugs
        with suppress_stderr():
            mic = sr.Microphone()
            source = mic.__enter__()
            
        try:
            # Adjust for ambient noise at the start of the session
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
            
            while self._running:
                if self._current_state != VoiceState.IDLE:
                    time.sleep(0.1)
                    continue

                try:
                    # Listen for a short phrase
                    audio = self.recognizer.listen(source, timeout=2.0, phrase_time_limit=3.0)
                    
                    try:
                        text = self.recognizer.recognize_google(audio).lower()
                        print(f"[VoiceEngine Debug] Heard while IDLE: '{text}'")
                        
                        wake_words = [self.wake_word, "can you hear", "can you hear me now", "can u hear me"]
                        if any(w in text for w in wake_words):
                            self._handle_wake_word(source)
                    except sr.UnknownValueError:
                        pass
                    except sr.RequestError as e:
                        print(f"[VoiceEngine] Google STT error: {e}")
                except sr.WaitTimeoutError:
                    pass  # Nobody spoke, loop again
                except Exception as e:
                    print(f"[VoiceEngine] Listen error: {e}")
                    time.sleep(1)
        finally:
            mic.__exit__(None, None, None)

    def _handle_wake_word(self, source):
        """Called when wake word is detected. Re-uses the open mic source."""
        self._set_state(VoiceState.LISTENING)
        
        try:
            # Listen up to 10s for the question
            audio = self.recognizer.listen(source, timeout=4.0, phrase_time_limit=10.0)
                
            self._set_state(VoiceState.THINKING)
            text = self.recognizer.recognize_google(audio)
            
            # Send the transcribed text back to the UI
            self.transcript_ready.emit(text)
            
        except sr.WaitTimeoutError:
            self._set_state(VoiceState.IDLE)
        except sr.UnknownValueError:
            self._set_state(VoiceState.IDLE)
        except Exception as e:
            print(f"[VoiceEngine] Command STT error: {e}")
            self._set_state(VoiceState.IDLE)
