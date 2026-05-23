## Running

```bash
$ colcon build
$ source install/setup.bash
$ ros2 run antenna_pkg antenna
```

colcon build; source install/setup.zsh; PORT_OVERRIDE=/tmp/ttyACM9 ros2 run antenna_pkg antenna

ros2 topic pub -1 /antenna/from_base astra_msgs/msg/AntennaControl "{command: 1, angle: 1.0}"
