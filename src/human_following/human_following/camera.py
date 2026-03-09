import cv2

class Camera:
    def __init__(self, source=0, width=640, height=480):
        self.source = source
        self.width = width
        self.height = height
        self.cap = None
        self._connect()

    def _connect(self):
        self.cap = cv2.VideoCapture(self.source)
        
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap.set(cv2.CAP_PROP_FPS, 30)

    def read(self):
        if self.cap is None or not self.cap.isOpened():
            return False, None
        
        return self.cap.read()

    def reconnect(self):
        self.release()
        self._connect()

    def release(self):
        if self.cap:
            self.cap.release()
            self.cap = None
