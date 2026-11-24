import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera')
        self.device = self.declare_parameter('device', '/dev/video0').value
        self.width = self.declare_parameter('width', 320).value
        self.height = self.declare_parameter('height', 240).value
        self.fps = self.declare_parameter('fps', 15).value
        self.fourcc = self.declare_parameter('fourcc', '').value
        self.exposure = self.declare_parameter('exposure', -1).value
        if min(self.width, self.height, self.fps) <= 0 or len(self.fourcc) not in (0, 4):
            raise ValueError('camera needs positive dimensions and fps, and a four-character fourcc')
        self.frame_id = f'{self.get_name()}_optical_frame'
        self.bridge = CvBridge()
        self.raw = self.create_publisher(Image, 'image_raw', qos_profile_sensor_data)
        self.jpeg = self.create_publisher(CompressedImage, 'image_raw/compressed', qos_profile_sensor_data)
        self.capture = None
        self.retry_at = 0.0
        self.create_timer(1 / self.fps, self.publish_frame)

    def open_camera(self):
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            return None
        if self.fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        if self.exposure >= 0:
            capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            capture.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
        return capture

    def publish_frame(self):
        if self.capture is None:
            if time.monotonic() < self.retry_at:
                return
            self.capture = self.open_camera()
            if self.capture is None:
                self.retry(f'Cannot open {self.device}')
                return
        ok, frame = self.capture.read()
        if not ok:
            self.release_camera()
            self.retry(f'No frames from {self.device}')
            return
        raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw.header.stamp = self.get_clock().now().to_msg()
        raw.header.frame_id = self.frame_id
        self.raw.publish(raw)
        jpeg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        jpeg.header = raw.header
        self.jpeg.publish(jpeg)

    def retry(self, message):
        self.retry_at = time.monotonic() + 2.0
        self.get_logger().warning(f'{message}, retrying', throttle_duration_sec=10.0)

    def release_camera(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None


def main():
    rclpy.init()
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.release_camera()
            node.destroy_node()
        rclpy.try_shutdown()
