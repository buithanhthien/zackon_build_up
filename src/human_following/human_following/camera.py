"""Colour capture with per-frame aligned metric depth for RealSense sources."""
import cv2
import numpy as np


class Camera:
    def __init__(self, source='realsense', width=640, height=480, fps=30,
                 serial='', pixel_format='YUYV'):
        self.source = source
        self.width, self.height, self.fps = width, height, fps
        self.serial, self.pixel_format = serial, pixel_format
        self.cap = self.pipeline = None
        self.depth_m = self.intrinsics = None
        self.requires_depth = source == "realsense"
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
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self.pipeline = pipeline
            profile = pipeline.start(config)
            self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            self.align = rs.align(rs.stream.color)
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

    def deproject(self, pixel, depth):
        return self.rs.rs2_deproject_pixel_to_point(self.intrinsics, pixel, depth)

    def read(self):
        # Clear metadata before every acquisition: never reuse an older depth frame.
        self.depth_m = self.intrinsics = None
        if self.pipeline is not None:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            frames = self.align.process(frames)
            colour = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if colour and depth:
                data = np.asanyarray(depth.get_data())
                colour_shape = np.asanyarray(colour.get_data()).shape[:2]
                if data.shape == colour_shape:
                    self.depth_m = data.astype(np.float32) * self.depth_scale
                    self.intrinsics = depth.profile.as_video_stream_profile().intrinsics
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
            try:
                pipeline.stop()
            except RuntimeError:
                pass  # start() may have failed before the pipeline was running.
        if self.cap is not None:
            cap, self.cap = self.cap, None
            cap.release()
