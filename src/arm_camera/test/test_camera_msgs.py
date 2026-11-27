import cv2
import numpy as np
from builtin_interfaces.msg import Time

from arm_camera.camera_node import compressed_msg, image_msg, is_url, windows_host


def test_is_url():
    assert is_url('http://1.2.3.4:8081/stream.mjpg') and is_url('RTSP://cam/x')
    assert not is_url('/dev/video0')


def test_windows_host_is_default_gateway(tmp_path):
    route = tmp_path / 'route'
    route.write_text(
        'Iface\tDestination\tGateway \tFlags\n'
        'eth0\t005018AC\t00000000\t0001\n'
        'eth0\t00000000\t015018AC\t0003\n')
    assert windows_host(str(route)) == '172.24.80.1'
    assert windows_host(str(tmp_path / 'missing')) == '127.0.0.1'


def test_image_msg_layout():
    frame = np.zeros((4, 6, 3), dtype=np.uint8)
    frame[1, 2] = (1, 2, 3)
    msg = image_msg(frame, Time(sec=5), 'cam')
    assert (msg.height, msg.width, msg.step, msg.encoding) == (4, 6, 18, 'bgr8')
    assert msg.header.frame_id == 'cam' and msg.header.stamp.sec == 5
    assert bytes(msg.data) == frame.tobytes()


def test_compressed_msg_is_decodable_jpeg():
    frame = np.full((48, 64, 3), 200, dtype=np.uint8)
    msg = compressed_msg(frame, Time(), 'cam', 80)
    assert msg.format == 'jpeg'
    decoded = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == frame.shape
