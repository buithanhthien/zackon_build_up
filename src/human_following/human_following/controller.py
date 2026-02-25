class MotionController:
    def __init__(self, linear_speed=0.3, angular_speed=0.5):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        
    def compute_velocity(self, zone):
        if zone == "left":
            return self.linear_speed, self.angular_speed * 0.3
        elif zone == "middle":
            return self.linear_speed, 0.0
        elif zone == "right":
            return self.linear_speed, -self.angular_speed * 0.3
        else:
            return 0.0, 0.0
