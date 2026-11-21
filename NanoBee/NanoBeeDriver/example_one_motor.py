"""Move one motor (joint 0 = X driver) back and forth.

Run from this folder with the board on USB:
    python example_one_motor.py
"""

from nanobee import NanoBee

PORT = "COM6"  # USB serial port of the board (Device Manager > Ports)
JOINT = 0

arm = NanoBee(port=PORT, config="config.yaml")
# Wi-Fi instead of USB: arm = NanoBee("192.168.4.1", config="config.yaml")
print("status:", arm.status())

try:
    arm.zero()  # current position becomes 0 deg
    arm.enable()

    arm.move(JOINT, 90, speed=30, wait=True)
    print("at 90 deg:", arm.angles()[JOINT])

    arm.move(JOINT, -45, speed=60, accel=120, wait=True)
    print("at -45 deg:", arm.angles()[JOINT])

    for _ in range(3):
        arm.move(JOINT, 10, speed=45, relative=True, wait=True)
    print("after 3 x +10 deg:", arm.angles()[JOINT])

    arm.move(JOINT, 0, speed=30, wait=True)
    print("back at 0:", arm.angles()[JOINT])
except KeyboardInterrupt:
    arm.halt()
    print("halted")
finally:
    arm.disable()
    arm.close()
