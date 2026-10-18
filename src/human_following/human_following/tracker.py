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
    def __init__(self, frame_width=640, frame_height=480, horizontal_fov=54.2,
                 desired_distance=1.0, depth_min=0.3, depth_max=3.0,
                 max_linear=0.2, max_angular=0.5):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = frame_width / 2
        self.center_y = frame_height / 2
        
        horizontal_fov_rad = math.radians(horizontal_fov)
        self.focal_length_px = frame_width / (2 * math.tan(horizontal_fov_rad / 2))
        
        self.desired_distance = desired_distance
        if not (all(math.isfinite(value) for value in
                    (depth_min, depth_max, desired_distance, max_linear, max_angular))
                and 0 < depth_min < depth_max and desired_distance > 0
                and max_linear > 0 and max_angular > 0):
            raise ValueError("Invalid following distance, depth range or speed limits")
        self.depth_min, self.depth_max = depth_min, depth_max
        self.depth_history = []
        self.depth_target = None
        self.measurement_status = 'searching'
        self.angle_threshold = 0.1
        self.angle_deadzone = 0.01
        self.vertical_margin = 10
        
        self.max_angular = max_angular
        self.max_linear = max_linear
        
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
            
        visible = [t for t in self.tracks if t.lost_frames == 0]
        if not visible:
            self.lock_candidate_id = self.lock_candidate_start_time = None
            return
        if self.locked_id is not None:
            return
        closest_track = min(visible, key=lambda t: abs((t.bbox[0] + t.bbox[2]) / 2 - self.center_x))
        
        if self.lock_candidate_id != closest_track.id:
            self.lock_candidate_id = closest_track.id
            self.lock_candidate_start_time = current_time
        elif current_time - self.lock_candidate_start_time >= self.lock_duration:
            if self.locked_id != closest_track.id:
                self.locked_id = closest_track.id
                
    def _stop(self, tracked, status):
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.measurement_status = status
        if status != 'depth_warming':
            self.depth_history.clear()
        return 0.0, 0.0, tracked, None

    def _depth(self, bbox, depth):
        if depth is None or depth.shape != (self.frame_height, self.frame_width):
            return None
        x1, y1, x2, y2 = map(float, bbox)
        # Central torso, excluding silhouette edges, head and legs. Bound sampling
        # cost to ~400 pixels regardless of camera resolution.
        left = max(0, int(x1 + .3 * (x2 - x1)))
        right = min(self.frame_width, int(x1 + .7 * (x2 - x1)))
        top = max(0, int(y1 + .2 * (y2 - y1)))
        bottom = min(self.frame_height, int(y1 + .6 * (y2 - y1)))
        if right <= left or bottom <= top:
            return None
        roi = depth[top:bottom:max(1, (bottom-top)//20),
                    left:right:max(1, (right-left)//20)]
        valid = roi[np.isfinite(roi) & (roi >= self.depth_min) & (roi <= self.depth_max)]
        if valid.size < max(12, roi.size * .5):
            return None
        median = float(np.median(valid))
        # Reject mixed foreground/background surfaces; isolated outliers do not
        # move the median. No temporal hole filling.
        if float(np.percentile(valid, 75) - np.percentile(valid, 25)) > .25:
            return None
        return median, [(left + right - 1) / 2, (top + bottom - 1) / 2]

    def get_human_position(self, results, current_time, depth_m=None,
                           deproject=None, camera_to_base=None, require_depth=False):
        if not results or len(results[0].boxes) == 0:
            self.update_tracks([], current_time)
            return self._stop([], 'searching')
        detections = [box.xyxy[0].cpu().numpy() for box in results[0].boxes]
        self.update_tracks(detections, current_time)
        tracked = [(t.id, t.bbox) for t in self.tracks if t.lost_frames == 0]
        if not tracked:
            return self._stop([], 'searching')
        if self.locked_id is not None:
            target = next((t for t in tracked if t[0] == self.locked_id), None)
            if target is None:
                # Retain identity while briefly occluded; never follow its stale box.
                if not any(t.id == self.locked_id for t in self.tracks):
                    self.locked_id = None
                return self._stop(tracked, 'target_lost')
        else:
            target = max(tracked, key=lambda t: (t[1][2]-t[1][0]) * (t[1][3]-t[1][1]))
        track_id, bbox = target
        if self.depth_target != track_id:
            self.depth_history.clear()
            self.angular_pid.reset()
            self.linear_pid.reset()
            self.depth_target = track_id

        if require_depth:
            sample = self._depth(bbox, depth_m)
            if sample is None or deproject is None:
                return self._stop(tracked, 'depth_invalid')
            if camera_to_base is None:
                return self._stop(tracked, 'tf_unavailable')
            z, pixel = sample
            if self.depth_history and abs(z - self.depth_history[-1]) > .35:
                self.depth_history.clear()
            self.depth_history = (self.depth_history + [z])[-3:]
            if len(self.depth_history) < 3:
                return self._stop(tracked, 'depth_warming')
            point = np.asarray(deproject(pixel, float(np.median(self.depth_history))))
            position = camera_to_base @ np.append(point, 1.0)
            if not np.all(np.isfinite(position)) or position[0] <= 0:
                return self._stop(tracked, 'depth_invalid')
            x, y, height = map(float, position[:3])
            distance = math.hypot(x, y)
            self.measurement_status = 'depth_valid'
        else:
            x1, y1, x2, y2 = bbox
            if y1 <= self.vertical_margin or y2 >= self.frame_height-self.vertical_margin or y2 <= y1:
                return self._stop(tracked, 'estimate_invalid')
            theta = math.atan2((x1+x2)/2-self.center_x, self.focal_length_px)
            distance = 1.7 * self.focal_length_px / (y2-y1)
            x, y, height = distance*math.cos(theta), -distance*math.sin(theta), 0.0
            self.measurement_status = 'estimated'
        angle = math.atan2(y, x)  # base_link: x forward, y left
        if abs(angle) <= self.angle_deadzone:
            angular = 0.0
            self.angular_pid.reset()
        else:
            angular = self.angular_pid.compute(angle, current_time)
        error = distance - self.desired_distance
        if error > .05:
            linear = self.linear_pid.compute(error, current_time) * max(0, 1-abs(angle)/.3)
        else:
            linear = 0.0
            self.linear_pid.reset()
        return float(linear), float(angular), tracked, dict(
            x=x, y=y, z=height, distance=distance, angle=angle, frame_id='base_link')
