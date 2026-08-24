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

# Gipformer STT imports (Offline Vietnamese ASR)
try:
    import sherpa_onnx
    import soundfile as sf
    GIPFORMER_AVAILABLE = True
except ImportError:
    GIPFORMER_AVAILABLE = False
    print("[VoiceEngine] Warning: sherpa-onnx not installed. Run: pip install sherpa-onnx soundfile")

# Supertonic TTS imports
try:
    from supertonic import TTS as SupertonicTTS
    SUPERTONIC_AVAILABLE = True
except ImportError:
    SUPERTONIC_AVAILABLE = False
    print("[VoiceEngine] Warning: supertonic not installed. Run: pip install supertonic")

# STT Configuration
USE_GIPFORMER = True  # Use Gipformer offline STT
GIPFORMER_MODEL_DIR = os.path.expanduser("~/.cache/gipformer")  # Model download location

# TTS Configuration
SUPERTONIC_VOICE = "M1"  # Options: M1-M5, F1-F5 (F1 is female voice, similar to HoaiMy)
SUPERTONIC_LANG = "vi"   # Vietnamese
SUPERTONIC_STEPS = 8     # Quality: 5 (low) to 12 (high), default 8
SUPERTONIC_SPEED = 1.7   # Speed: 0.7 (slow) to 2.0 (fast) - Increased for faster speech

MIC_DEVICE_PRIORITY = [
    "pipewire",       # PipeWire routes all physical mics (USB, 3.5mm jack)
    "pulse",
    "default",
]
MIC_JACK_KEYWORDS = ["usb2.0", "usb audio", "usb_audio", "usb", "c-media", "cmedia"]  # USB mic identifiers
MIC_JACK_EXCLUDE  = ["hdmi", "iec958", "spdif"]  # never treat these as mic input
MIC_FORCE_INDEX = None
MIC_SAMPLE_RATE = 16000

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
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 300
        self.recognizer.pause_threshold  = 1.5

        # Initialize Gipformer STT (Offline Vietnamese)
        self._gipformer_recognizer = None
        if USE_GIPFORMER and GIPFORMER_AVAILABLE:
            try:
                print("[VoiceEngine] Initializing Gipformer STT (offline Vietnamese ASR)...")
                self._init_gipformer()
                if self._gipformer_recognizer:
                    print("[VoiceEngine] ✅ Gipformer STT ready (offline mode)")
                else:
                    print("[VoiceEngine] ⚠️ Gipformer initialization failed")
            except Exception as e:
                print(f"[VoiceEngine] Gipformer error: {e}")
                self._gipformer_recognizer = None

        # Initialize Supertonic TTS
        self._supertonic_tts = None
        self._supertonic_voice_style = None
        if SUPERTONIC_AVAILABLE:
            try:
                print("[VoiceEngine] Initializing Supertonic TTS (first run downloads ~400MB model)...")
                self._supertonic_tts = SupertonicTTS(auto_download=True)
                self._supertonic_voice_style = self._supertonic_tts.get_voice_style(voice_name=SUPERTONIC_VOICE)
                print(f"[VoiceEngine] Supertonic TTS initialized with voice: {SUPERTONIC_VOICE}")
            except Exception as e:
                print(f"[VoiceEngine] Failed to initialize Supertonic: {e}")
                self._supertonic_tts = None

        self._tts_queue = queue.Queue()
        threading.Thread(target=self._tts_worker, daemon=True).start()
    
    def _init_gipformer(self):
        """Initialize Gipformer ASR model"""
        try:
            # Check if model exists, if not, download it
            model_dir = GIPFORMER_MODEL_DIR
            encoder_path = os.path.join(model_dir, "encoder-epoch-35-avg-6.onnx")
            decoder_path = os.path.join(model_dir, "decoder-epoch-35-avg-6.onnx")
            joiner_path = os.path.join(model_dir, "joiner-epoch-35-avg-6.onnx")
            tokens_path = os.path.join(model_dir, "tokens.txt")
            
            if not all(os.path.exists(p) for p in [encoder_path, decoder_path, joiner_path, tokens_path]):
                print("[VoiceEngine] Gipformer model not found, downloading from HuggingFace...")
                self._download_gipformer_model()
            
            # Create Gipformer RNNT recognizer
            self._gipformer_recognizer = sherpa_onnx.OfflineRecognizer.from_transducer(
                encoder=encoder_path,
                decoder=decoder_path,
                joiner=joiner_path,
                tokens=tokens_path,
                num_threads=2,
                sample_rate=16000,
                feature_dim=80,
                decoding_method="greedy_search",
            )
            
        except Exception as e:
            print(f"[VoiceEngine] Failed to initialize Gipformer: {e}")
            self._gipformer_recognizer = None
    
    def _download_gipformer_model(self):
        """Download Gipformer model from HuggingFace"""
        try:
            from huggingface_hub import hf_hub_download
            
            model_dir = GIPFORMER_MODEL_DIR
            os.makedirs(model_dir, exist_ok=True)
            
            repo_id = "g-group-ai-lab/gipformer-65M-rnnt"
            files = [
                "encoder-epoch-35-avg-6.onnx",
                "decoder-epoch-35-avg-6.onnx", 
                "joiner-epoch-35-avg-6.onnx",
                "tokens.txt"
            ]
            
            print(f"[VoiceEngine] Downloading Gipformer model (~280MB) to {model_dir}...")
            for filename in files:
                print(f"  Downloading {filename}...")
                hf_hub_download(
                    repo_id=repo_id,
                    filename=filename,
                    local_dir=model_dir,
                    local_dir_use_symlinks=False
                )
            print("[VoiceEngine] ✅ Gipformer model downloaded successfully")
            
        except ImportError:
            print("[VoiceEngine] ⚠️ huggingface-hub not installed. Run: pip install huggingface-hub")
            raise
        except Exception as e:
            print(f"[VoiceEngine] ⚠️ Failed to download Gipformer model: {e}")
            raise

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
            if jack_idx is not None:
                candidates.append(jack_idx)
            for name in MIC_DEVICE_PRIORITY:
                idx = next((i for i, n in enumerate(available) if name.lower() in n.lower()), None)
                if idx is not None and idx not in candidates:
                    candidates.append(idx)
            candidates.append(None)

        audio = None
        try:
            for idx in candidates:
                try:
                    mic = sr.Microphone(device_index=idx, sample_rate=MIC_SAMPLE_RATE)
                    with mic as source:
                        try:
                            with _suppress_stderr():
                                self._set_state(VoiceState.LISTENING)
                                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                        except Exception as e:
                            print(f"[VoiceEngine] mic index={idx} failed: {e}")
                            continue
                        print(f"[VoiceEngine] using mic index={idx} ({available[idx] if idx is not None else 'default'})")
                        print(f"[VoiceEngine] energy_threshold={self.recognizer.energy_threshold:.1f}, listening...")
                        try:
                            audio = self.recognizer.listen(source, timeout=5.0, phrase_time_limit=15.0)
                            duration = len(audio.frame_data) / (audio.sample_rate * audio.sample_width)
                            print(f"[VoiceEngine] Audio duration: {duration:.2f}s")
                            print("[VoiceEngine] audio captured, processing...")
                        except sr.WaitTimeoutError:
                            print("[VoiceEngine] timeout — no speech detected")
                        break  # mic worked; stop trying candidates
                except Exception as e:
                    print(f"[VoiceEngine] mic index={idx} open failed: {e}")
            else:
                print("[VoiceEngine] no usable microphone found")
                return

            if audio is None:
                return

            self._set_state(VoiceState.THINKING)
            
            # Gipformer offline STT only
            text = None
            if self._gipformer_recognizer:
                try:
                    print("[VoiceEngine] Processing with Gipformer (offline)...")
                    text = self._recognize_with_gipformer(audio)
                    if text:
                        print(f"[VoiceEngine] Gipformer recognized: '{text}'")
                    else:
                        print("[VoiceEngine] Gipformer could not understand audio")
                except Exception as e:
                    print(f"[VoiceEngine] Gipformer failed: {e}")
            
            else:
                print("[VoiceEngine] Gipformer is not available")
            
            if text:
                self.transcript_ready.emit(text)
                
        finally:
            self.state_changed.emit("")
            self._listen_lock.release()
    
    def _recognize_with_gipformer(self, audio_data):
        """Recognize speech using Gipformer offline ASR"""
        try:
            import numpy as np

            # Convert AudioData to 16 kHz mono float32
            raw_data = audio_data.get_raw_data(
                convert_rate=16000,
                convert_width=2
            )

            audio_array = (np.frombuffer(raw_data, dtype=np.int16).astype(np.float32) / 32768.0)

            # Create offline recognition stream
            stream = self._gipformer_recognizer.create_stream()

            # Give the complete utterance to the offline recognizer
            stream.accept_waveform(
                16000,
                audio_array
            )

            # Decode
            self._gipformer_recognizer.decode_streams([stream])

            # Get result
            text = stream.result.text.strip()

            return text if text else None

        except Exception as e:
            print(f"[VoiceEngine] Gipformer recognition error: {e}")
            return None

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
            tmp_path = None
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                    tmp_path = f.name
                
                # Synthesize with Supertonic
                if self._supertonic_tts and self._supertonic_voice_style:
                    wav, duration = self._supertonic_tts.synthesize(
                        text=text,
                        voice_style=self._supertonic_voice_style,
                        lang=SUPERTONIC_LANG,
                        total_steps=SUPERTONIC_STEPS,
                        speed=SUPERTONIC_SPEED,
                        verbose=False
                    )
                    # Save the audio
                    self._supertonic_tts.save_audio(wav, tmp_path)
                    
                    if not self._stop_flag.is_set():
                        self._set_state(VoiceState.SPEAKING)
                        self._play_audio(tmp_path)
                else:
                    print("[TTS] Supertonic not available, skipping synthesis")
            except Exception as e:
                print(f"[TTS] synthesis failed: {e}")
            finally:
                if tmp_path:
                    try:
                        os.unlink(tmp_path)
                    except Exception:
                        pass
            self._tts_queue.task_done()
            if self._tts_queue.empty():
                self._set_state("")

    def _play_audio(self, path: str):
        with self._play_lock:
            # Use aplay for WAV files (more common on Linux) or fallback to mpg123
            try:
                proc = subprocess.Popen(
                    ['aplay', '-q', path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except FileNotFoundError:
                # Fallback to play (from sox package)
                proc = subprocess.Popen(
                    ['play', '-q', path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            
            while proc.poll() is None:
                if self._stop_flag.is_set():
                    proc.kill()
                    break
                time.sleep(0.05)
