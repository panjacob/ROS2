# 2. Development environment

ROS 2 runs best on Linux. The repository has been tested on native Ubuntu 26.04
with ROS 2 Lyrical and MoveIt 2. Ubuntu 24.04 with ROS 2 Jazzy is another option.
On Windows, use WSL2 with Ubuntu and follow the same Linux build steps.

## Native Ubuntu (without WSL)

Skip section 2.1. Install ROS 2 for your Ubuntu release using the
official installation guide for
[Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) or
[Lyrical](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html),
then follow section 2.3 to build this repository. Install `colcon` and `rosdep`
if they are missing: `sudo apt install python3-colcon-common-extensions
python3-rosdep`. Run `sudo rosdep init` once on a new ROS installation, then
`rosdep update`.
If ROS 2 is already installed, source its setup file in each new terminal, for example
`source /opt/ros/lyrical/setup.bash` on the tested machine or
`source /opt/ros/jazzy/setup.bash` for Jazzy. Confirm with `echo $ROS_DISTRO`.
Install MoveIt 2 with `sudo apt install ros-$ROS_DISTRO-moveit` after sourcing ROS;
the workspace's other ROS dependencies are listed in the package manifests and
installed by `rosdep` in section 2.3.
You do not need USB passthrough for simulation, MoveIt planning or Execute.

## 2.1 Install WSL2 + Ubuntu (Windows, PowerShell as admin)

```powershell
wsl --install -d Ubuntu-24.04
wsl --update
```

Reboot, open "Ubuntu 24.04", create a user. WSLg (built in to Windows 11) displays GUI apps such as
RViz.

> Keep the workspace **inside the Linux filesystem** (e.g. `~/ros2_arm`), not on `/mnt/d/...` –
> builds on `/mnt` are very slow. Clone this repo there, or symlink it. VS Code: install the
> "WSL" extension and run `code .` from the Ubuntu shell.

## 2.2 Install ROS 2 Jazzy (inside Ubuntu)

Follow the official guide: <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>.
Summary:

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# Add the ROS 2 apt repository via the ros2-apt-source package (required, do not skip):
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-moveit ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  python3-serial
sudo rosdep init || true
rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check: `ros2 run demo_nodes_cpp talker` in one terminal, `ros2 run demo_nodes_py listener` in another.

## 2.3 Build this workspace

```bash
cd ~/ros2_arm
source /opt/ros/lyrical/setup.bash  # use jazzy instead if that is what you installed
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch arm_start arm.launch.py
```

In each later terminal, source ROS and `install/setup.bash` again before launching
the arm or using ROS CLI commands.

Optionally, source the workspace automatically in new terminals (adjust the
path to your workspace). Place this after the line in `~/.bashrc` that sources
`/opt/ros/<distro>/setup.bash`; it takes effect in new terminals or after
`source ~/.bashrc`:

```bash
echo '[ -f ~/ros2_arm/install/setup.bash ] && source ~/ros2_arm/install/setup.bash' >> ~/.bashrc
```

After pulling changes or adding new packages or files (meshes, launch files,
URDF/Xacro), rebuild the whole workspace from its root with
`colcon build --symlink-install` and source `install/setup.bash` again.
`--symlink-install` only reflects edits to files that are already installed.
Typical symptoms of a stale build are
`PackageNotFoundError: package 'arm_moveit_config' not found` from
`ros2 launch arm_start arm.launch.py`, or missing arm meshes in RViz. To check
for missing system dependencies, run
`rosdep check --from-paths src --ignore-src -r`.

Open http://localhost:8080. Enter five
joint targets, click **Plan joint motion**, inspect the RViz preview, then click
**Execute previewed plan**. STOP interrupts the motion. These commands work on
native Ubuntu and WSL2.

The launch opens RViz with the repository's configuration and the planned
trajectory display. For a display-only preview with joint sliders instead,
stop the launch with Ctrl+C and run `ros2 launch arm_description display.launch.py`.

## 2.4 Connecting to the TinyBee

### Option A – Wi-Fi (simplest with WSL2)

FluidNC exposes a telnet server on port 23. No USB passthrough needed.
Set in [fluidnc_bridge.yaml](../src/arm_start/config/fluidnc_bridge.yaml):

```yaml
transport: tcp
host: fluidnc.local   # or the board IP
```

If `fluidnc.local` does not resolve inside WSL, use the IP address shown in the FluidNC
startup log (`$SS`) or your router.

### Option B – USB serial via usbipd-win

The TinyBee uses a CH340 USB-UART. In Windows PowerShell (admin):

```powershell
winget install usbipd
usbipd list                          # find the CH340 BUSID, e.g. 2-3
usbipd bind --busid 2-3              # once
usbipd attach --wsl --busid 2-3      # needed after every replug / reboot
```

`ros2 launch arm_start arm.launch.py use_hardware:=true` does the attach automatically
([attach_board.sh](../src/arm_start/scripts/attach_board.sh): if `/dev/ttyUSB0` is missing it runs
`usbipd.exe attach --wsl --hardware-id 1a86:7523`, no admin needed). Manual: `ros2 run arm_start attach_board.sh`.

In Ubuntu:

```bash
ls /dev/ttyUSB*                      # expect /dev/ttyUSB0
sudo usermod -aG dialout $USER       # then restart the WSL shell
```

While attached to WSL the port is **not** available to Windows tools (FluidTerm, WebInstaller).
Detach with `usbipd detach --busid 2-3`.

> Opening the serial port toggles DTR and **reboots the ESP32**. The bridge tolerates this,
> but expect ~2 s of boot messages after start.

### 2.4.1 USB camera (Microsoft LifeCam VX-6000)

The camera (`045e:00f4`, Sonix SN9C20x + OV9650) has no driver in Windows 11 nor in Microsoft's
WSL kernel, so it is used from WSL with an out-of-tree `gspca_sn9c20x` module:

```powershell
usbipd bind --hardware-id 045e:00f4          # once, PowerShell as admin
```

```bash
tools/wsl_camera_driver.sh                   # once, and again after every WSL kernel update
```

`arm.launch.py` then attaches the camera automatically (same `attach_board.sh`) and starts
`arm_camera` (`camera:=false` to disable). The image is on the web page and on `/camera/image_raw`.

- Over usbipd only the camera's JPEG mode delivers complete frames (the node forces `fourcc: JPEG`),
  and only 320×240 streams reliably (steady ~17 fps); 640×480 stalls or stutters, so 320×240 is the
  default. Change it with `camera_width:=640 camera_height:=480` (fine on a native Linux host, where
  `gspca_sn9c20x` ships with the stock kernel and no attach/driver build is needed).
- The leftover Windows LifeCam files (`C:\Windows\vVX6000.exe` = tray app, `VX6000Xp.sys`) are not a
  usable driver: no INF, missing `VX6KCamd.sys`, and an XP-era stream-class driver that Windows 11
  blocks while Memory Integrity is on. Capturing on Windows is therefore not an option for this camera.
- Auto exposure drops to ~3 fps in a dim room, so exposure is fixed: `camera_exposure:=1000`
  (≈17 fps). Lower = faster but darker (300 ≈ 28 fps); `-1` = auto. Add light rather than exposure.
- Check: `v4l2-ctl --list-devices`, `ros2 topic hz /camera/image_raw`.

### 2.4.2 Phone camera (Telescope)

The phone is the second camera, through [Telescope](https://github.com/LunarKittyy/Telescope)
(phone app + Windows desktop app, used unmodified). WSL can't open Windows webcams, so
`arm.launch.py` starts a small bridge on Windows (WSL interop, `py.exe`) that reads Telescope's
virtual camera with ffmpeg and serves it as MJPEG; the `camera2` node in WSL reads that stream.

1. In Telescope on Windows: pair the phone, set up the virtual camera and press Start.
2. Windows needs Python 3 (`py.exe`) and `ffmpeg` on PATH. Then just `ros2 launch arm_start arm.launch.py`;
   the bridge stops with the launch. Its log lines start with `[camera2_bridge]`.

   Arguments: `camera2:=false` (no phone camera), `camera2_name:=<Windows camera name or its start>`
   (default `Telescope`), `camera2_bridge:=false` (start the bridge yourself). By hand, in
   PowerShell from the repository folder:

   ```powershell
   py src\arm_camera\windows\windows_camera_bridge.py --list    # camera names: expect "Telescope (...)"
   py src\arm_camera\windows\windows_camera_bridge.py --width 1280 --fps 30
   ```

   Defaults: 640 px wide, 15 fps, port 8081; also `--quality 2..31`, `--camera`.
   It listens only on Windows' WSL adapter (not the LAN, there is no login). With
   `networkingMode=mirrored` use `--host 127.0.0.1` and
   `camera2_device:=http://127.0.0.1:8081/stream.mjpg`.
3. `{windows_host}` in `camera2_device` becomes the WSL default gateway (= Windows). Image on the
   web page and on `/camera2/image_raw` (`frame_id` `camera2_link`).

- `camera2 ... not available, retrying` / `Connection timed out`: the bridge isn't running
  (look for `[camera2_bridge]` errors) or the firewall blocks it. Test from WSL with
  `curl http://$(ip route show default | cut -d' ' -f3):8081/frame.jpg`; if it times out while the
  bridge runs, allow the port on the WSL adapter once (PowerShell as admin):
  `New-NetFirewallRule -DisplayName "ROS2 camera bridge (WSL)" -Direction Inbound -Protocol TCP -LocalPort 8081 -InterfaceAlias "vEthernet (WSL (Hyper-V firewall))" -Action Allow`
- Exposure, focus and lens are set in Telescope; `camera_exposure` etc. only apply to the USB camera.
- Check: `ros2 topic hz /camera2/image_raw` (≈ the bridge's `--fps`).

## 2.5 Useful tools

| Tool | Use |
|------|-----|
| `rqt`, `rqt_graph` | Inspect topics/nodes |
| `ros2 topic echo /joint_states` | Check feedback |
| `check_urdf`, `xacro arm.urdf.xacro > /tmp/arm.urdf` | Validate model |
| FluidNC WebUI (`http://fluidnc.local`) | Jogging, config upload, console |
| `screen /dev/ttyUSB0 115200` or `python3 -m serial.tools.miniterm` | Raw FluidNC console |
| Git + Git LFS | Version STL/STEP files |

## 2.6 Native Windows / Docker (alternatives)

* ROS 2 has a native Windows build, but MoveIt 2 and many tools are Linux-first – not recommended.
* Docker (`osrf/ros:jazzy-desktop`) works for headless nodes; GUI and USB need extra setup.
* A Raspberry Pi 5 / mini PC with Ubuntu can later run the bridge next to the arm while your
  PC runs RViz/MoveIt over the network (same `ROS_DOMAIN_ID`).
