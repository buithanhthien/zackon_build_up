from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class HumanDetector:
    def __init__(self):
        pkg_dir = get_package_share_directory('human_following')
        model_path = os.path.join(pkg_dir, 'yolov8n.pt')
        self.model = YOLO(model_path)

    def detect(self, frame):
        results = self.model(frame, classes=[0], verbose=False)
        return results
