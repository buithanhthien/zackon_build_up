"""CPU classifier worker; never blocks the ROS safety executor."""
import hashlib
import json
import math
from pathlib import Path
import threading
import time

MODEL_DIR = Path(__file__).resolve().parents[1] / 'models' / 'elevator_detect'
DEFAULT_CONFIDENCE_THRESHOLD = 0.85
MAX_RESULT_AGE = 1.0


def classify(probabilities, threshold=DEFAULT_CONFIDENCE_THRESHOLD):
    if type(threshold) not in (int, float) or not math.isfinite(threshold) or not 0.5 < threshold <= 1:
        raise ValueError('Ngưỡng nhận diện phải lớn hơn 0.5 và không quá 1.')
    if len(probabilities) != 2 or not all(math.isfinite(p) and 0 <= p <= 1 for p in probabilities):
        return 'unknown', 0.0
    if abs(sum(probabilities) - 1) > 1e-4:
        return 'unknown', 0.0
    index = max(range(2), key=lambda i: probabilities[i])
    confidence = probabilities[index]
    return (('available', 'unavailable')[index] if confidence >= threshold else 'unknown'), confidence


def freshness_error(result_stamp, frame_stamp, now):
    """Explain freshness separately from a model's uncertain classification."""
    if frame_stamp <= 0:
        return 'Chưa nhận được frame camera hợp lệ.'
    if not 0 <= now - frame_stamp <= MAX_RESULT_AGE:
        return f'Frame camera đã cũ hoặc sai thời gian (tuổi {now - frame_stamp:.2f} s).'
    if result_stamp <= 0:
        return 'Đang chờ kết quả nhận diện đầu tiên.'
    if not 0 <= now - result_stamp <= MAX_RESULT_AGE:
        return f'Kết quả nhận diện đã cũ hoặc sai thời gian (tuổi {now - result_stamp:.2f} s).'
    return None


class ElevatorDetector:
    def __init__(self, directory=MODEL_DIR):
        self.lock = threading.Lock()
        self.frame = None
        self.result = ('unknown', 0.0, 0.0)
        self.error = 'Đang nạp model / chờ camera.'
        self.ready = False
        self.enabled = False
        self.generation = 0
        self.threshold = DEFAULT_CONFIDENCE_THRESHOLD
        self.probabilities = None
        self.inference_seconds = None
        self.input_error = ''
        self.closed = threading.Event()
        self.directory = Path(directory)
        self.thread = threading.Thread(target=self._run, daemon=True, name='elevator-detector')
        self.thread.start()

    def submit(self, rgb, stamp):
        with self.lock:
            self.frame = (rgb, stamp)
            self.input_error = ''

    def reject_frame(self, reason):
        with self.lock:
            self.frame = None
            self.result = ('unknown', 0.0, 0.0)
            self.input_error = reason
            self.probabilities = None
            self.inference_seconds = None
            self.generation += 1

    def snapshot(self):
        with self.lock:
            return self.ready, self.error, self.result, self.frame[1] if self.frame else 0.0

    def enable(self):
        with self.lock:
            self.result = ('unknown', 0.0, 0.0)
            self.probabilities = None
            self.inference_seconds = None
            self.enabled = True
            self.generation += 1

    def disable(self):
        with self.lock:
            self.enabled = False
            self.result = ('unknown', 0.0, 0.0)
            self.probabilities = None
            self.inference_seconds = None
            self.generation += 1

    def describe(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if not self.ready:
                return self.error
            if self.input_error:
                return self.input_error
            if not self.enabled:
                return 'Nhận diện chưa bật; dùng kiểm tra tại chỗ hoặc bắt đầu test.'
            label, confidence, stamp = self.result
            frame_stamp = self.frame[1] if self.frame else 0.0
            reason = freshness_error(stamp, frame_stamp, now)
            if self.probabilities is None:
                scores = 'Chưa có điểm dự đoán.'
            else:
                scores = (f'available: {self.probabilities[0]:.1%} | '
                          f'unavailable: {self.probabilities[1]:.1%}.')
            if reason is None:
                reason = ('unknown: điểm cao nhất chưa đạt ngưỡng.' if label == 'unknown'
                          else f'Kết quả: {label}.')
            latency = (f' Suy luận: {self.inference_seconds * 1000:.0f} ms.'
                       if self.inference_seconds is not None else '')
            return f'{reason}\n{scores} Ngưỡng: {self.threshold:.0%}.{latency}'

    def _run(self):
        try:
            metadata = json.loads((self.directory / 'manifest.json').read_text())
            if metadata['license_status'] != 'authorized':
                raise ValueError('Chưa xác nhận quyền sử dụng/phân phối model trong manifest.json.')
            threshold = metadata.get('confidence_threshold', DEFAULT_CONFIDENCE_THRESHOLD)
            classify([0.5, 0.5], threshold)  # validate before loading the model
            with self.lock:
                self.threshold = threshold
            path = self.directory / 'best.pt'
            if hashlib.sha256(path.read_bytes()).hexdigest() != metadata['sha256']:
                raise ValueError('SHA256 model không khớp manifest.')
            import torch
            from torch import nn
            from torchvision import transforms as T
            from torchvision.models import mobilenet_v3_small
            from PIL import Image
            checkpoint = torch.load(path, map_location='cpu', weights_only=True)
            if checkpoint['classes'] != ['available', 'unavailable'] or checkpoint['image_size'] != 224:
                raise ValueError('Class labels hoặc image_size checkpoint không hợp lệ.')
            model = mobilenet_v3_small(weights=None)
            model.classifier[-1] = nn.Linear(model.classifier[-1].in_features, 2)
            model.load_state_dict(checkpoint['state_dict'], strict=True)
            model.eval()
            preprocess = T.Compose([T.Resize((224, 224)), T.ToTensor(), T.Normalize(
                [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
            with torch.inference_mode():
                model(torch.zeros(1, 3, 224, 224))
            with self.lock:
                self.ready, self.error = True, ''
            last = 0.0
            while not self.closed.wait(0.05):
                with self.lock:
                    frame, enabled = self.frame, self.enabled
                    generation = self.generation
                if not enabled or frame is None or frame[1] <= last:
                    continue
                rgb, last = frame
                started = time.monotonic()
                with torch.inference_mode():
                    probabilities = model(preprocess(Image.fromarray(rgb)).unsqueeze(0)).softmax(1)[0]
                scores = probabilities.tolist()
                label, confidence = classify(scores, threshold)
                with self.lock:
                    # Discard work interrupted by Stop, rejected frames or a new session.
                    if self.enabled and self.frame is not None and generation == self.generation:
                        self.result = (label, confidence, last)
                        self.probabilities = tuple(scores)
                        self.inference_seconds = time.monotonic() - started
        except Exception as exc:
            with self.lock:
                self.ready, self.error = False, f'Lỗi detector: {exc}'
                self.result = ('error', 0.0, 0.0)

    def close(self):
        self.closed.set()
