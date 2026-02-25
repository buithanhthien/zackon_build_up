class HumanTracker:
    def __init__(self, frame_width):
        self.frame_width = frame_width
        self.zone_width = frame_width / 3
        
    def get_zone(self, bbox):
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        
        if center_x < self.zone_width:
            return "left"
        elif center_x < 2 * self.zone_width:
            return "middle"
        else:
            return "right"
    
    def get_human_position(self, results):
        if not results or len(results[0].boxes) == 0:
            return None
            
        largest_box = max(results[0].boxes, key=lambda b: (b.xyxy[0][2] - b.xyxy[0][0]) * (b.xyxy[0][3] - b.xyxy[0][1]))
        bbox = largest_box.xyxy[0].cpu().numpy()
        return self.get_zone(bbox)
