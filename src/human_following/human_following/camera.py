"""Colour camera sources: RealSense SDK or an explicitly selected V4L2/URL source."""
import cv2


class Camera:
    def __init__(self, source='realsense', width=640, height=480, fps=30,
                 serial='', pixel_format='YUYV'):
        self.source = source
        self.width, self.height, self.fps = width, height, fps
        self.serial, self.pixel_format = serial, pixel_format
        self.cap = self.pipeline = None
        try:
            self._connect()
        except BaseException:
            self.release()
            raise

    def _connect(self):
        if self.source == 'realsense':
            try:
                import pyrealsense2 as rs
            except ImportError as exc:
                raise RuntimeError('RealSense requires pyrealsense2 in the ROS Python environment; '
                                   'or set camera_url to the colour /dev/v4l/by-id/... device') from exc
            self.rs = rs
            pipeline = rs.pipeline()
            config = rs.config()
            if self.serial:
                config.enable_device(self.serial)
            config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
            pipeline.start(config)
            self.pipeline = pipeline
            return
        source = self.source
        if isinstance(source, str) and source.isdecimal():
            source = int(source)
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open colour camera: {self.source}')
        if self.pixel_format:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read(self):
        if self.pipeline is not None:
            import numpy as np
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            colour = frames.get_color_frame()
            frame = np.asanyarray(colour.get_data()) if colour else None
            ok = colour is not None and bool(colour)
        elif self.cap is not None:
            ok, frame = self.cap.read()
        else:
            return False, None
        # A depth/metadata node is not a valid colour source.
        if not ok or frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            return False, None
        return True, frame

    def release(self):
        if self.pipeline is not None:
            pipeline, self.pipeline = self.pipeline, None
            pipeline.stop()
        if self.cap is not None:
            cap, self.cap = self.cap, None
            cap.release()
