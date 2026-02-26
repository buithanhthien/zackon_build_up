import cv2

class Camera:
    def __init__(self, source=0, width=640, height=480):
        self.source = source
        self.width = width
        self.height = height
        self.cap = None
        self._connect()

    def _connect(self):
        if isinstance(self.source, str) and self.source.startswith('rtsp://'):
            self.cap = cv2.VideoCapture(self.source, cv2.CAP_FFMPEG)
        else:
            self.cap = cv2.VideoCapture(self.source)
        
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer
            self.cap.set(cv2.CAP_PROP_FPS, 30)  # Set FPS
            
            # For network streams, reduce latency
            if isinstance(self.source, str) and ('http' in self.source or 'rtsp' in self.source):
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    def read(self):
        if self.cap is None or not self.cap.isOpened():
            return False, None
        
        # Flush old frames from buffer for network streams
        if isinstance(self.source, str) and ('http' in self.source or 'rtsp' in self.source):
            for _ in range(2):  # Skip 2 old frames
                self.cap.grab()
        
        return self.cap.read()

    def reconnect(self):
        self.release()
        self._connect()

    def release(self):
        if self.cap:
            self.cap.release()
            self.cap = None
