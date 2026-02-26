import time

class PIDController:
    def __init__(self, kp=1.0, ki=1.0, kd=1.0, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
    def compute(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()
        
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0
        
        dt = current_time - self.prev_time
        if dt <= 0.0:
            return 0.0
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.prev_error) / dt
        
        # Compute output
        output = p_term + i_term + d_term
        
        # Apply limits
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # Update state
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
