"""Serve a Windows webcam (e.g. Telescope's phone camera) as MJPEG over HTTP, for ROS 2 in WSL.

WSL2 can't open Windows webcams, so this runs on Windows: the installed ffmpeg reads the camera
through DirectShow and every client of http://<WSL adapter>:8081/stream.mjpg gets the same frames.
It listens only on Windows' WSL virtual adapter by default, so the LAN can't see the camera.
Standard library only; needs ffmpeg on PATH. arm.launch.py starts it (camera2_bridge:=true);
by hand, from the repository folder in PowerShell:

  py src\\arm_camera\\windows\\windows_camera_bridge.py --list     # camera names
  py src\\arm_camera\\windows\\windows_camera_bridge.py            # "Telescope", 640 px wide, port 8081
  py src\\arm_camera\\windows\\windows_camera_bridge.py --camera "POCO F9 Ultra" --width 1280 --fps 30

GET /stream.mjpg   multipart MJPEG
GET /frame.jpg     latest frame
"""

import argparse
import re
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SOI, EOI = b'\xff\xd8', b'\xff\xd9'


def wsl_adapter_ip():
    """Windows' address on the WSL virtual network, so only WSL (not the LAN) reaches the bridge."""
    cmd = ("(Get-NetIPAddress -AddressFamily IPv4 -InterfaceAlias 'vEthernet (WSL*)' "
           "-ErrorAction SilentlyContinue | Select-Object -First 1).IPAddress")
    try:
        out = subprocess.run(['powershell', '-NoProfile', '-Command', cmd],
                             capture_output=True, text=True, timeout=15).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out or None


def list_cameras(ffmpeg):
    out = subprocess.run([ffmpeg, '-hide_banner', '-list_devices', 'true', '-f', 'dshow', '-i', 'dummy'],
                         capture_output=True, text=True, errors='replace').stderr
    return re.findall(r'"([^"]+)" \(video\)', out)


def resolve_camera(wanted, cameras):
    """Exact name, else the first one starting with it: Windows 11 appends e.g. " (Windows Virtual Camera)"."""
    if wanted in cameras:
        return wanted
    return next((c for c in cameras if c.lower().startswith(wanted.lower())), None)


class Frames:
    def __init__(self):
        self._cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.stamp = 0.0

    def put(self, jpeg):
        with self._cond:
            self.jpeg, self.seq = jpeg, self.seq + 1
            self.stamp = time.monotonic()
            self._cond.notify_all()

    def latest(self):
        with self._cond:
            return self.jpeg if time.monotonic() - self.stamp < 2.0 else None

    def wait_next(self, seq, timeout):
        with self._cond:
            self._cond.wait_for(lambda: self.seq != seq, timeout)
            return self.seq, self.jpeg


def capture(ffmpeg, args, frames):
    vf = f'fps={args.fps},scale={args.width}:-2' if args.width > 0 else f'fps={args.fps}'
    while True:
        cameras = list_cameras(ffmpeg)
        camera = resolve_camera(args.camera, cameras)
        if camera is None:
            print(f'Camera "{args.camera}" not found. Available: {cameras or "none"}. '
                  'Set up the virtual camera in Telescope; retrying in 5 s.', flush=True)
            time.sleep(5.0)
            continue
        cmd = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'dshow', '-rtbufsize', '64M',
               '-i', f'video={camera}', '-vf', vf, '-f', 'mjpeg', '-q:v', str(args.quality), '-']
        print(f'Opening camera "{camera}"...', flush=True)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        threading.Thread(target=lambda p=proc: [print('ffmpeg:', l.decode(errors='replace').rstrip(), flush=True)
                                                for l in p.stderr], daemon=True).start()
        buf, count = b'', 0
        while chunk := proc.stdout.read1(65536):
            buf += chunk
            while (start := buf.find(SOI)) >= 0 and (end := buf.find(EOI, start + 2)) >= 0:
                frames.put(buf[start:end + 2])
                buf = buf[end + 2:]
                count += 1
                if count == 1:
                    print('Streaming.', flush=True)
        proc.wait()
        print(f'Camera stopped (ffmpeg exit {proc.returncode}); is Telescope streaming? Retrying in 2 s.',
              flush=True)
        time.sleep(2.0)


def make_handler(frames):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            path = self.path.split('?')[0]
            if path == '/frame.jpg':
                jpeg = frames.latest()
                if jpeg is None:
                    self.send_error(503, 'no frame yet')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.end_headers()
                self.wfile.write(jpeg)
            elif path in ('/', '/stream.mjpg'):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                seq = frames.seq
                try:
                    while True:
                        next_seq, jpeg = frames.wait_next(seq, timeout=2.0)
                        if next_seq == seq:
                            break
                        seq = next_seq
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                         + str(len(jpeg)).encode() + b'\r\n\r\n' + jpeg + b'\r\n')
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
            else:
                self.send_error(404)
    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', default='Telescope',
                    help='DirectShow camera name or its start (default: Telescope)')
    ap.add_argument('--list', action='store_true', help='list cameras and exit')
    ap.add_argument('--width', type=int, default=640, help='output width, height keeps aspect (0 = as is)')
    ap.add_argument('--fps', type=float, default=15, help='output frame rate')
    ap.add_argument('--quality', type=int, default=5, help='JPEG quality 2 (best) .. 31')
    ap.add_argument('--host', default='wsl',
                    help='address to listen on: wsl = only the WSL virtual adapter (default), '
                         '127.0.0.1 for WSL mirrored networking, 0.0.0.0 = whole LAN (no login!)')
    ap.add_argument('--port', type=int, default=8081)
    args = ap.parse_args()

    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg is None:
        sys.exit('ffmpeg not found on PATH (e.g. choco install ffmpeg / winget install ffmpeg)')
    if args.list:
        print('\n'.join(list_cameras(ffmpeg)) or 'no cameras found')
        return
    if args.host == 'wsl':
        args.host = wsl_adapter_ip()
        if args.host is None:
            sys.exit('No WSL network adapter found: start WSL first, or pass --host 127.0.0.1 '
                     '(mirrored networking)')

    frames = Frames()
    try:
        server = ThreadingHTTPServer((args.host, args.port), make_handler(frames))
    except OSError as exc:
        # Usually a bridge started by hand already serves the camera; that one is used instead.
        print(f'Port {args.port} on {args.host} is busy ({exc}); is a bridge already running? Exiting.',
              flush=True)
        return
    threading.Thread(target=capture, args=(ffmpeg, args, frames), daemon=True).start()
    server.daemon_threads = True
    print(f'Serving http://{args.host}:{args.port}/stream.mjpg (Ctrl+C to stop)', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
