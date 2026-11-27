"""Camera -> ROS 2 images (OpenCV).

Publishes
  camera/image_raw             sensor_msgs/Image (bgr8)
  camera/image_raw/compressed  sensor_msgs/CompressedImage (jpeg) - used by the web page

device is either a V4L2 device (USB camera) or a network stream URL (http://, https://, rtsp://),
e.g. the phone camera served from Windows by windows/windows_camera_bridge.py. In a URL
{windows_host} is replaced by the Windows host's address as seen from WSL (default gateway).
For URLs fourcc/exposure/gain are not used and width/height are taken from the stream.

The camera may be unplugged, not attached to WSL or the stream not running yet; the node keeps retrying.
If setup_command is set (arm.launch.py: arm_start's attach_camera.sh) it is run whenever the device
can't be opened, to load the driver and attach the camera to WSL (backing off after failures).

LifeCam VX-6000 over WSL usbipd: only the camera's own JPEG mode delivers complete frames
(raw Bayer/I420 frames arrive mostly black), and only 320x240 streams reliably (steady ~17 fps);
640x480 stalls or stutters. Its auto exposure drops to ~3 fps in a dim room,
so exposure/gain are fixed by default (exposure 300 ~ 28 fps, 800 ~ 21 fps, 1500 ~ 11 fps);
set exposure:=-1 for auto.
"""

import shlex
import shutil
import socket
import struct
import subprocess
import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

DEFAULT_DEVICE = '/dev/v4l/by-id/usb-045e_USB20_Camera-video-index0'
URL_SCHEMES = ('http://', 'https://', 'rtsp://')
SETUP_TIMEOUT = 1800.0  # a first-time driver build takes minutes
SETUP_RETRY_MIN, SETUP_RETRY_MAX = 10.0, 300.0


def is_url(device):
    return device.lower().startswith(URL_SCHEMES)


def windows_host(route_file='/proc/net/route'):
    """Default gateway: under WSL2 NAT networking this is the Windows host."""
    try:
        with open(route_file) as f:
            for line in f.readlines()[1:]:
                fields = line.split()
                if len(fields) > 2 and fields[1] == '00000000':
                    return socket.inet_ntoa(struct.pack('<L', int(fields[2], 16)))
    except (OSError, ValueError):
        pass
    return '127.0.0.1'


def resolve_device(device):
    if '{windows_host}' in device:
        return device.replace('{windows_host}', windows_host())
    return device


def image_msg(frame, stamp, frame_id):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    return msg


def compressed_msg(frame, stamp, frame_id, quality):
    ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        return None
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    msg.data = jpeg.tobytes()
    return msg


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera')
        self.device = resolve_device(self.declare_parameter('device', DEFAULT_DEVICE).value)
        self.width = int(self.declare_parameter('width', 320).value)
        self.height = int(self.declare_parameter('height', 240).value)
        self.fps = float(self.declare_parameter('fps', 15.0).value)
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 80).value)
        self.fourcc = self.declare_parameter('fourcc', 'JPEG').value
        self.exposure = int(self.declare_parameter('exposure', 1000).value)
        self.gain = int(self.declare_parameter('gain', 28).value)
        self.setup_command = str(self.declare_parameter('setup_command', '').value).strip()
        self._setup_retry = SETUP_RETRY_MIN
        self._next_setup = 0.0
        self._raw_pub = self.create_publisher(
            Image, 'camera/image_raw', qos_profile_sensor_data)
        self._jpeg_pub = self.create_publisher(
            CompressedImage, 'camera/image_raw/compressed', qos_profile_sensor_data)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self):
        if is_url(self.device):
            cap = cv2.VideoCapture(self.device, cv2.CAP_FFMPEG, [
                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000])
            if not cap.isOpened():
                cap.release()
                return None
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            return cap
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._set_controls()
        return cap

    def _set_controls(self):
        if self.exposure < 0:
            controls = ['gain_automatic=1']
        else:
            controls = ['gain_automatic=0', f'exposure={self.exposure}', f'gain={self.gain}']
        tool = shutil.which('v4l2-ctl')
        if tool is None:
            self.get_logger().warn('v4l2-ctl not found (apt install v4l-utils); '
                                   'camera exposure left unchanged')
            return
        result = subprocess.run(
            [tool, '-d', self.device, '-c', ','.join(controls)],
            capture_output=True, text=True, timeout=5.0)
        if result.returncode != 0:
            self.get_logger().warn(f'Setting {controls} failed: {result.stderr.strip()}')

    def _setup(self):
        """Run setup_command (load driver / attach to WSL); True if it succeeded."""
        if not self.setup_command or is_url(self.device) or time.monotonic() < self._next_setup:
            return False
        log = self.get_logger()
        try:
            result = subprocess.run(shlex.split(self.setup_command), stdin=subprocess.DEVNULL,
                                    capture_output=True, text=True, timeout=SETUP_TIMEOUT)
            output = (result.stdout + result.stderr).strip()
            ok = result.returncode == 0
        except (OSError, subprocess.TimeoutExpired) as exc:
            output, ok = str(exc), False
        for line in output.splitlines():
            (log.info if ok else log.warn)(line)
        if ok:
            self._setup_retry = SETUP_RETRY_MIN
        else:
            self._setup_retry = min(self._setup_retry * 2.0, SETUP_RETRY_MAX)
        self._next_setup = time.monotonic() + (SETUP_RETRY_MIN if ok else self._setup_retry)
        return ok

    def _loop(self):
        log = self.get_logger()
        cap = None
        while self._running and rclpy.ok():
            if cap is None:
                cap = self._open()
                if cap is None and self._setup():
                    cap = self._open()
                if cap is None:
                    log.warn(f'Camera {self.device} not available, retrying...',
                             throttle_duration_sec=10.0)
                    time.sleep(2.0)
                    continue
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                log.info(f'Camera {self.device} opened ({w}x{h})')
            ok, frame = cap.read()
            if not ok:
                log.warn(f'Camera {self.device} stopped delivering frames, reopening')
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            stamp = self.get_clock().now().to_msg()
            if self._raw_pub.get_subscription_count():
                self._raw_pub.publish(image_msg(frame, stamp, self.frame_id))
            if self._jpeg_pub.get_subscription_count():
                msg = compressed_msg(frame, stamp, self.frame_id, self.jpeg_quality)
                if msg is not None:
                    self._jpeg_pub.publish(msg)
        if cap is not None:
            cap.release()

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
