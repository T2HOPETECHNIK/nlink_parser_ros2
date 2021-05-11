## nlink_parser_ros2

This is a port of the [nlink_parser](https://github.com/nooploop-dev/nlink_parser) to run using ros2 foxy. \
*NOTE* : Currently ONLY the code for linktrack_aoa node has been ported

### Dependencies

This package depends on the ros2 port of the [serial library](https://github.com/wjwwood/serial) which can be found [here](https://github.com/SunnyApp-Robotics/serial/tree/ros2).

```
cd <path_to_your_ros2_ws>/src
git clone git@github.com:SunnyApp-Robotics/serial.git
cd serial
git checkout ros2
```

This package also depends on the cutom ros2 msgs for nlink_parser which are placed in the `nlink_parser_ros2_interfaces` package.

### Build

```
colcon build --packages-up-to nlink_parser_ros2
```

### Launch

Since only the linktrack_aoa node has been ported currently, the launch file only contains that node.

```
ros2 launch nlink_parser_ros2 linktrack_aoa.launch.py
```