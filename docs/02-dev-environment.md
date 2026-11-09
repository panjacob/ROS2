# 2. Development environment

ROS 2 runs best on Linux. On Windows use **WSL2 with Ubuntu 24.04** and **ROS 2 Jazzy
Jalisco** (LTS, supported until 2029, MoveIt 2 binaries available). If a newer LTS
distribution is available and MoveIt 2 supports it, you may use that instead – adjust
`jazzy` in the commands below.

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
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch arm_description display.launch.py
```

In RViz: set *Fixed Frame* = `world`, *Add → RobotModel* (Description Topic `/robot_description`),
*Add → TF*. Save the config (`File → Save Config As`) into `src/arm_description/rviz/` if you
want it loaded automatically later.

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
