# AI should not modify this file. It is for humans only.

ros2 launch arm_start arm.launch.py use_hardware:=false
ros2 launch arm_start arm.launch.py use_hardware:=true

## Start symulacji
`source install/setup.bash` -> done in `install/setup.bash`
```bash
colcon build --symlink-install
ros2 launch arm_start arm.launch.py
```

Po odpaleniu powinien pojawić się panel na localhost:8080 z planowaniem i wykonywaniem ruchu.
