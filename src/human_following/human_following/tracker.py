import math
import numpy as np
from .pid_controller import PIDController

class Track:
    def __init__(self, track_id, bbox):
        self.id = track_id
        self.bbox = bbox
        self.age = 0
        self.lost_frames = 0
        self.state = np.array([bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]], dtype=np.float32)
        
    def predict(self):
        self.age += 1
        self.lost_frames += 1
        return self.state
        
    def update(self, bbox):
        self.bbox = bbox
        self.state = np.array([bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]], dtype=np.float32)
        self.lost_frames = 0

class HumanTracker:
    def __init__(self, frame_width=640, frame_height=480, horizontal_fov=68.4):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = frame_width / 2
        self.center_y = frame_height / 2
        
        horizontal_fov_rad = math.radians(horizontal_fov)
        self.focal_length_px = frame_width / (2 * math.tan(horizontal_fov_rad / 2))
        
        self.desired_distance = 1.5
        self.angle_threshold = 0.1
        self.angle_deadzone = 0.01
        self.vertical_margin = 10
        
        self.max_angular = 1.0
        self.max_linear = 1.0
        
        self.angular_pid = PIDController(kp=2.0, ki=0.02, kd=0.25, output_limits=(-self.max_angular, self.max_angular))
        self.linear_pid = PIDController(kp=2.0, ki=0.0, kd=0.08, output_limits=(0.0, self.max_linear))
        
        self.tracks = []
        self.next_id = 1
        self.max_lost_frames = 30
        self.iou_threshold = 0.3
        
        self.locked_id = None
        self.lock_candidate_id = None
        self.lock_candidate_start_time = None
        self.lock_duration = 5.0
        
    def iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        
        return inter / union if union > 0 else 0
        
    def update_tracks(self, detections, current_time):
        if len(detections) == 0:
            for track in self.tracks:
                track.predict()
            self.tracks = [t for t in self.tracks if t.lost_frames < self.max_lost_frames]
            return []
            
        if len(self.tracks) == 0:
            for det in detections:
                self.tracks.append(Track(self.next_id, det))
                self.next_id += 1
            return [(t.id, t.bbox) for t in self.tracks]
            
        cost_matrix = np.zeros((len(self.tracks), len(detections)))
        for i, track in enumerate(self.tracks):
            for j, det in enumerate(detections):
                cost_matrix[i, j] = 1 - self.iou(track.bbox, det)
                
        matched_tracks = set()
        matched_dets = set()
        
        for _ in range(min(len(self.tracks), len(detections))):
            min_cost = np.inf
            min_i, min_j = -1, -1
            for i in range(len(self.tracks)):
                if i in matched_tracks:
                    continue
                for j in range(len(detections)):
                    if j in matched_dets:
                        continue
                    if cost_matrix[i, j] < min_cost:
                        min_cost = cost_matrix[i, j]
                        min_i, min_j = i, j
                        
            if min_cost < (1 - self.iou_threshold):
                self.tracks[min_i].update(detections[min_j])
                matched_tracks.add(min_i)
                matched_dets.add(min_j)
            else:
                break
                
        for i, track in enumerate(self.tracks):
            if i not in matched_tracks:
                track.predict()
                
        for j, det in enumerate(detections):
            if j not in matched_dets:
                self.tracks.append(Track(self.next_id, det))
                self.next_id += 1
                
        self.tracks = [t for t in self.tracks if t.lost_frames < self.max_lost_frames]
        
        self._update_lock_status(current_time)
        
        return [(t.id, t.bbox) for t in self.tracks]
        
    def _update_lock_status(self, current_time):
        if len(self.tracks) == 0:
            self.lock_candidate_id = None
            self.lock_candidate_start_time = None
            return
            
        closest_track = min(self.tracks, key=lambda t: abs((t.bbox[0] + t.bbox[2]) / 2 - self.center_x))
        
        if self.lock_candidate_id != closest_track.id:
            self.lock_candidate_id = closest_track.id
            self.lock_candidate_start_time = current_time
        elif current_time - self.lock_candidate_start_time >= self.lock_duration:
            if self.locked_id != closest_track.id:
                self.locked_id = closest_track.id
                
    def get_human_position(self, results, current_time):
        if not results or len(results[0].boxes) == 0:
            self.angular_pid.reset()
            self.linear_pid.reset()
            self.update_tracks([], current_time)
            return (0.0, 0.0, [])
            
        detections = [box.xyxy[0].cpu().numpy() for box in results[0].boxes]
        tracked = self.update_tracks(detections, current_time)
        
        if len(tracked) == 0:
            self.angular_pid.reset()
            self.linear_pid.reset()
            return (0.0, 0.0, [])
            
        if self.locked_id:
            target = next((t for t in tracked if t[0] == self.locked_id), None)
            if not target:
                target = tracked[0]
        else:
            target = max(tracked, key=lambda t: (t[1][2] - t[1][0]) * (t[1][3] - t[1][1]))
            
        track_id, bbox = target
        x_min, y_min, x_max, y_max = bbox
        
        if y_min <= self.vertical_margin or y_max >= self.frame_height - self.vertical_margin:
            self.angular_pid.reset()
            self.linear_pid.reset()
            return (0.0, 0.0, tracked)
            
        bbox_center_x = (x_min + x_max) / 2
        delta_px = bbox_center_x - self.center_x
        theta = math.atan2(delta_px, self.focal_length_px)
        
        if abs(theta) <= self.angle_deadzone:
            angular_z = 0.0
            self.angular_pid.reset()
        else:
            angular_z = float(-self.angular_pid.compute(theta))
            
        bbox_height = y_max - y_min
        estimated_distance = (1.7 * self.focal_length_px) / bbox_height
        distance_error = estimated_distance - self.desired_distance
        
        if distance_error > 0:
            linear_x = float(self.linear_pid.compute(distance_error))
            alignment_factor = max(0, 1 - abs(theta)/0.3)
            linear_x *= alignment_factor
        else:
            linear_x = 0.0
            self.linear_pid.reset()
            
        return (float(linear_x), float(angular_z), tracked)
