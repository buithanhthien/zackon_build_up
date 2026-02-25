import math

class HumanTracker:
    def __init__(self, frame_width=640, frame_height=480, horizontal_fov=68.4):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = frame_width / 2
        self.center_y = frame_height / 2
        
        # Camera parameters
        horizontal_fov_rad = math.radians(horizontal_fov)
        self.focal_length_px = frame_width / (2 * math.tan(horizontal_fov_rad / 2))
        
        # Control parameters
        self.K_ang = 1.0
        self.K_lin = 0.5
        
        self.desired_distance = 1.5
        self.angle_threshold = 0.1
        self.angle_deadzone = 0.02
        self.vertical_margin = 10
        
        # Velocity limits
        self.max_angular = 1.5
        self.max_linear = 0.6
        
    def get_human_position(self, results):
        """Returns (linear_velocity, angular_velocity) tuple"""
        if not results or len(results[0].boxes) == 0:
            return (0.0, 0.0)
        
        # Get largest bounding box
        largest_box = max(results[0].boxes, 
                         key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) * (b.xyxy[0][3] - b.xyxy[0][1]))
        bbox = largest_box.xyxy[0].cpu().numpy()
        
        x_min, y_min, x_max, y_max = bbox
        
        # Check vertical violation
        if y_min <= self.vertical_margin or y_max >= self.frame_height - self.vertical_margin:
            return (0.0, 0.0)
        
        # Compute bounding box center
        bbox_center_x = (x_min + x_max) / 2
        bbox_center_y = (y_min + y_max) / 2
        
        # Compute horizontal offset
        delta_px = bbox_center_x - self.center_x
        
        # Compute rotation angle using pinhole camera model
        theta = math.asin(max(-1.0, min(1.0, delta_px / self.focal_length_px)))

        # Compute angular velocity
        if abs(theta) < self.angle_deadzone:
            angular_z = 0.0
        else:
            angular_z = float(-self.K_ang * theta)
            angular_z = max(-self.max_angular, min(self.max_angular, angular_z))
        
        # Compute linear velocity (only if aligned)
        if abs(theta) < self.angle_threshold:
            # Calculate distance using pinhole camera model
            bbox_height = y_max - y_min
            estimated_distance = (1.7 * self.focal_length_px) / bbox_height
            
            linear_x = float(max(0.0, self.K_lin * (estimated_distance - self.desired_distance)))
        else:
            linear_x = 0.0
        
        return (float(linear_x), float(angular_z))
