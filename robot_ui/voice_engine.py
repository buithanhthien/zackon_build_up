import os
import glob
import queue
import re
import subprocess
import tempfile
import threading
import time

from contextlib import contextmanager
from language_config import get_language

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
ENGLISH_ZIPFORMER_MODEL_DIR = os.path.expanduser(
    "~/.cache/sherpa_onnx_models/sherpa-onnx-zipformer-small-en-2023-06-26"
)
# TTS Configuration
SUPERTONIC_VOICE = "F1"  # Options: M1-M5, F1-F5 (F1 is female voice, similar to HoaiMy)
SUPERTONIC_STEPS = 8     # Quality: 5 (low) to 12 (high), default 8
SUPERTONIC_SPEED = 1.7   # Speed: 0.7 (slow) to 2.0 (fast) - Increased for faster speech


# ============================================================
# Seeed Studio ReSpeaker Lite
#
# lsusb:
# ID 2886:0019 Seeed Technology Co., Ltd. ReSpeaker Lite
# ============================================================

SEEED_USB_VENDOR_ID = "2886"
SEEED_USB_PRODUCT_ID = "0019"

# ReSpeaker Lite hiện dùng PCM device 0
SEEED_PCM_DEVICE = 0

MIC_SAMPLE_RATE = 16000

# ============================================================
# Debug log
# ============================================================

VOICE_DEBUG = False


def _debug(message):
    if VOICE_DEBUG:
        print(message)

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

def _find_usb_audio_card(
    vendor_id: str,
    product_id: str
):
    """
    Tìm ALSA card thuộc đúng USB device theo VID/PID.

    Ví dụ ReSpeaker Lite:
        VID = 2886
        PID = 0019

    Trả về:
        số card ALSA, ví dụ 1

    Nếu không tìm thấy:
        None
    """

    vendor_id = vendor_id.lower()
    product_id = product_id.lower()

    card_paths = glob.glob(
        "/sys/class/sound/card[0-9]*"
    )

    for card_path in card_paths:

        card_name = os.path.basename(
            card_path
        )

        try:

            card_number = int(
                card_name.replace(
                    "card",
                    ""
                )
            )

        except ValueError:

            continue

        # Theo symlink từ ALSA card về USB device
        device_path = os.path.realpath(
            os.path.join(
                card_path,
                "device"
            )
        )

        current_path = device_path

        # Đi ngược lên các parent trong sysfs
        # cho tới USB device thật
        for _ in range(10):

            vendor_file = os.path.join(
                current_path,
                "idVendor"
            )

            product_file = os.path.join(
                current_path,
                "idProduct"
            )

            if (
                os.path.exists(vendor_file)
                and
                os.path.exists(product_file)
            ):

                try:

                    with open(
                        vendor_file,
                        "r",
                        encoding="utf-8"
                    ) as f:

                        detected_vendor = (
                            f.read()
                            .strip()
                            .lower()
                        )

                    with open(
                        product_file,
                        "r",
                        encoding="utf-8"
                    ) as f:

                        detected_product = (
                            f.read()
                            .strip()
                            .lower()
                        )

                except Exception:

                    break

                if (
                    detected_vendor == vendor_id
                    and
                    detected_product == product_id
                ):

                    return card_number

            parent = os.path.dirname(
                current_path
            )

            if parent == current_path:
                break

            current_path = parent

    return None

def _find_microphone_index_from_alsa_card(
    card_number: int,
    pcm_device: int = 0
):
    """
    Chuyển ALSA card/device thành device_index
    mà SpeechRecognition/PyAudio sử dụng.
    """

    with _suppress_stderr():

        microphones = (
            sr.Microphone.list_microphone_names()
        )

    target_hw = (
        f"hw:{card_number},{pcm_device}"
    )

    for index, name in enumerate(
        microphones
    ):

        if target_hw in name:

            return index, name

    return None, None

class VoiceEngine(QObject):
    state_changed    = pyqtSignal(str)
    transcript_ready = pyqtSignal(str)
    ui_command       = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._play_lock    = threading.Lock()
        self._stop_flag    = threading.Event()
        self._listen_lock  = threading.Lock()

        # Ngôn ngữ hiện tại của VoiceEngine
        self._language = get_language()

        self.recognizer = sr.Recognizer()
        self.recognizer.dynamic_energy_threshold = True
        self.recognizer.energy_threshold = 300
        self.recognizer.pause_threshold  = 1.5

        # Initialize Gipformer STT (Offline Vietnamese)
        self._gipformer_recognizer = None
        self._english_recognizer = None

        if GIPFORMER_AVAILABLE:

            if self._language == "vi":

                try:
                    self._init_gipformer()

                except Exception as e:
                    print(f"[VoiceEngine] Gipformer error: {e}")

            elif self._language == "en":
                try:

                    self._init_english_zipformer()

                except Exception as e:
                    print(f"[VoiceEngine] English Zipformer error: {e}")

        # Initialize Supertonic TTS
        self._supertonic_tts = None
        self._supertonic_voice_style = None
        if SUPERTONIC_AVAILABLE:
            try:
                _debug("[VoiceEngine] Initializing Supertonic TTS (first run downloads ~400MB model)...")
                self._supertonic_tts = SupertonicTTS(auto_download=True)
                self._supertonic_voice_style = self._supertonic_tts.get_voice_style(voice_name=SUPERTONIC_VOICE)
                print(f"[VoiceEngine] Supertonic TTS initialized with voice: {SUPERTONIC_VOICE}")
            except Exception as e:
                _debug(f"[VoiceEngine] Failed to initialize Supertonic: {e}")
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

    def _init_english_zipformer(self):
        """
        Khởi tạo Zipformer offline cho nhận dạng tiếng Anh.
        """

        try:

            model_dir = (
                ENGLISH_ZIPFORMER_MODEL_DIR
            )

            encoder_path = os.path.join(
                model_dir,
                "encoder-epoch-99-avg-1.onnx"
            )

            decoder_path = os.path.join(
                model_dir,
                "decoder-epoch-99-avg-1.onnx"
            )

            joiner_path = os.path.join(
                model_dir,
                "joiner-epoch-99-avg-1.onnx"
            )

            tokens_path = os.path.join(
                model_dir,
                "tokens.txt"
            )

            required_files = [
                encoder_path,
                decoder_path,
                joiner_path,
                tokens_path,
            ]

            missing_files = [
                path
                for path in required_files
                if not os.path.exists(path)
            ]

            if missing_files:

                print(
                    "[VoiceEngine] English Zipformer "
                    "model files are missing:"
                )

                for path in missing_files:
                    print(
                        f"  - {path}"
                    )

                self._english_recognizer = None

                return

            self._english_recognizer = (
                sherpa_onnx.OfflineRecognizer.from_transducer(
                    encoder=encoder_path,
                    decoder=decoder_path,
                    joiner=joiner_path,
                    tokens=tokens_path,
                    num_threads=2,
                    sample_rate=16000,
                    feature_dim=80,
                    decoding_method="greedy_search",
                )
            )

        except Exception as e:

            print(
                "[VoiceEngine] Failed to initialize "
                f"English Zipformer: {e}"
            )

            self._english_recognizer = None
    
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

        audio = None

        try:

            # ========================================================
            # 1. Tìm đúng ReSpeaker Lite bằng USB VID/PID
            # ========================================================

            card_number = _find_usb_audio_card(SEEED_USB_VENDOR_ID, SEEED_USB_PRODUCT_ID)

            if card_number is None:
                print("[VoiceEngine] ❌ Không tìm thấy ReSpeaker Lite USB 2886:0019")
                return

            _debug(
                "[VoiceEngine] ✅ ReSpeaker Lite USB found | "
                f"VID:PID="
                f"{SEEED_USB_VENDOR_ID}:"
                f"{SEEED_USB_PRODUCT_ID} | "
                f"ALSA card={card_number}"
            )

            # ========================================================
            # 2. Tìm device_index cho SpeechRecognition
            # ========================================================

            mic_index, mic_name = (_find_microphone_index_from_alsa_card(card_number, SEEED_PCM_DEVICE))

            if mic_index is None:

                print("[VoiceEngine] ❌ Tìm thấy USB ReSpeaker nhưng không tìm thấy ALSA input "
                    f"hw:{card_number},"
                    f"{SEEED_PCM_DEVICE}"
                )
                return

            _debug(
                "[VoiceEngine] ✅ ReSpeaker audio input | "
                f"index={mic_index} | "
                f"{mic_name}"
            )

            # ============================================================
            # Mở ReSpeaker Lite nhưng ẩn cảnh báo ALSA / JACK / PortAudio
            # ============================================================

            source = None
            mic = None

            try:

                # Ẩn cảnh báo ALSA / JACK ngay từ lúc PyAudio khởi tạo
                with _suppress_stderr():

                    mic = sr.Microphone(
                        device_index=mic_index,
                        sample_rate=MIC_SAMPLE_RATE
                    )

                    source = mic.__enter__()

                self._set_state(
                    VoiceState.LISTENING
                )

                # Cân chỉnh môi trường cũng có thể gọi audio backend
                with _suppress_stderr():

                    self.recognizer.adjust_for_ambient_noise(
                        source,
                        duration=0.5
                    )

                _debug(
                    "[VoiceEngine] using ReSpeaker Lite | "
                    f"USB={SEEED_USB_VENDOR_ID}:"
                    f"{SEEED_USB_PRODUCT_ID} | "
                    f"ALSA=hw:{card_number},"
                    f"{SEEED_PCM_DEVICE} | "
                    f"PyAudio index={mic_index}"
                )

                _debug(
                    "[VoiceEngine] "
                    f"energy_threshold="
                    f"{self.recognizer.energy_threshold:.1f}, "
                    "listening..."
                )

                try:

                    audio = self.recognizer.listen(
                        source,
                        timeout=5.0,
                        phrase_time_limit=15.0
                    )

                except sr.WaitTimeoutError:

                    print(
                        "[VoiceEngine] "
                        "timeout — no speech detected"
                    )

                    return

            finally:

                # Đóng audio device cũng có thể sinh cảnh báo ALSA/JACK
                if source is not None:

                    with _suppress_stderr():

                        try:

                            mic.__exit__(
                                None,
                                None,
                                None
                            )

                        except Exception:

                            pass

            if audio is None:
                return

            duration = (len(audio.frame_data)/(audio.sample_rate * audio.sample_width))

            _debug(
                "[VoiceEngine] "
                f"Audio duration: {duration:.2f}s"
            )

            _debug(
                "[VoiceEngine] "
                "audio captured, processing..."
            )

            self._set_state(VoiceState.THINKING)

            # ========================================================
            # 4. Chọn STT theo ngôn ngữ
            # ========================================================

            text = None

            # ========================================================
            # Tiếng Việt -> Gipformer
            # ========================================================

            if self._language == "vi":
                if self._gipformer_recognizer:

                    # print("[VoiceEngine] Processing Vietnamese with Gipformer...")

                    text = (self._recognize_with_gipformer(audio))

                    if text:
                        _debug("[VoiceEngine] Gipformer recognized: "
                            f"'{text}'"
                        )

                    else:
                        print("[VoiceEngine] Gipformer could not understand audio")

                else:

                    print("[VoiceEngine] Vietnamese STT is not available")

            # ========================================================
            # English -> Zipformer
            # ========================================================

            elif self._language == "en":

                if self._english_recognizer:

                    _debug("[VoiceEngine] Processing English with Zipformer...")

                    text = (self._recognize_with_english_zipformer(audio))

                    if text:
                        _debug(
                            "[VoiceEngine] Zipformer recognized: "
                            f"'{text}'"
                        )

                    else:
                        print("[VoiceEngine] Zipformer could not understand audio")

                else:
                    print("[VoiceEngine] English STT is not available")

            # ========================================================
            # 5. Chỉ gửi transcript đúng 1 lần
            # ========================================================

            if text:
                self.transcript_ready.emit(text)

        except Exception as e:

            print("[VoiceEngine] "
                f"Microphone error: {e}"
            )

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

    def _recognize_with_english_zipformer(
        self,
        audio_data
    ):
        try:

            import numpy as np

            # Chuyển AudioData về:
            # 16 kHz
            # mono
            # float32

            raw_data = (
                audio_data.get_raw_data(
                    convert_rate=16000,
                    convert_width=2
                )
            )

            audio_array = (
                np.frombuffer(
                    raw_data,
                    dtype=np.int16
                ).astype(np.float32)
                / 32768.0
            )

            if self._english_recognizer is None:
                return None

            stream = (
                self._english_recognizer.create_stream()
            )

            stream.accept_waveform(
                16000,
                audio_array
            )

            self._english_recognizer.decode_streams(
                [stream]
            )

            text = (
                stream.result.text.strip()
            )

            if not text:
                return None

            return text

        except Exception as e:

            print(
                "[VoiceEngine] English Zipformer "
                f"recognition error: {e}"
            )

            return None

    def set_language(self, language: str):

        if language not in (
            "vi",
            "en",
        ):

            print(
                "[VoiceEngine] "
                f"Ngôn ngữ không hợp lệ: {language}"
            )

            return False

        if language == self._language:

            return True

        # ============================================================
        # Dừng TTS đang chạy
        # ============================================================

        self.stop_speaking()

        # ============================================================
        # Đổi ngôn ngữ
        # ============================================================

        self._language = language

        print(
            "[VoiceEngine] "
            f"Language changed to: {language}"
        )

        # ============================================================
        # Tiếng Việt → Gipformer
        # ============================================================

        if language == "vi":

            # Giải phóng model English
            self._english_recognizer = None

            if self._gipformer_recognizer is None:

                print(
                    "[VoiceEngine] Loading "
                    "Vietnamese Gipformer..."
                )

                self._init_gipformer()

            if self._gipformer_recognizer:

                print(
                    "[VoiceEngine] ✅ Vietnamese STT ready"
                )

            else:

                print(
                    "[VoiceEngine] ❌ Vietnamese STT "
                    "failed to load"
                )

        # ============================================================
        # English → Zipformer
        # ============================================================

        elif language == "en":

            # Giải phóng model tiếng Việt
            self._gipformer_recognizer = None

            if self._english_recognizer is None:

                print(
                    "[VoiceEngine] Loading "
                    "English Zipformer..."
                )

                self._init_english_zipformer()

            if self._english_recognizer:

                print(
                    "[VoiceEngine] ✅ English STT ready"
                )

            else:

                print(
                    "[VoiceEngine] ❌ English STT "
                    "failed to load"
                )

        return True

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
        """
        Phát câu nói bằng ngôn ngữ hiện tại của VoiceEngine.
        """

        self.speak_in_language(
            text,
            self._language
        )


    def speak_in_language(
        self,
        text: str,
        language: str
    ):
        """
        Phát câu nói bằng một ngôn ngữ được chỉ định cụ thể.

        language:
            vi -> Tiếng Việt
            en -> English
        """

        if language not in (
            "vi",
            "en",
        ):

            print(
                f"[TTS] Invalid language: "
                f"{language}"
            )

            language = self._language

        # ============================================================
        # Làm sạch text trước khi đưa vào TTS
        # ============================================================

        text = re.sub(
            r'<[^>]+>',
            '',
            text
        )

        text = re.sub(
            r'\*\*([^*]+)\*\*',
            r'\1',
            text
        )

        text = re.sub(
            r'\*([^*]+)\*',
            r'\1',
            text
        )

        text = (
            text
            .replace('"', '')
            .replace('"', '')
            .replace('"', '')
        )

        text = re.sub(
            r'\.(?=[a-zA-Z])',
            ' chấm ',
            text
        )

        text = text.strip()

        if not text:
            return

        _debug(
            f"[TTS QUEUE] language={language} | "
            f"text={text}"
        )

        # Quan trọng:
        # lưu cả text và language vào queue
        self._tts_queue.put(
            (
                text,
                language
            )
        )

    def _set_state(self, state: str):
        self.state_changed.emit(state)

    def _tts_worker(self):
        while True:
            item = self._tts_queue.get()

            # ========================================================
            # Mỗi câu TTS đi kèm ngôn ngữ của chính câu đó
            # ========================================================

            if (isinstance(item, tuple) and len(item) == 2):
                text, language = item
            else:
                # Tương thích với queue cũ nếu có
                text = item
                language = self._language

            self._stop_flag.clear()

            tmp_path = None
            try:
                with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                    tmp_path = f.name
                
                # Synthesize with Supertonic
                if self._supertonic_tts and self._supertonic_voice_style:
                    wav, duration = (
                        self._supertonic_tts.synthesize(
                            text=text,
                            voice_style=self._supertonic_voice_style,
                            lang=language,
                            total_steps=SUPERTONIC_STEPS,
                            speed=SUPERTONIC_SPEED,
                            verbose=False
                        )
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
