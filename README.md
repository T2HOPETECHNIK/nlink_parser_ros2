## nlink_parser_ros2

This is a port of the [nlink_parser](https://github.com/nooploop-dev/nlink_parser) to run using ros2 foxy to use with the LinkTrack and LinkTrack AoA devices. \
*NOTE* : `tofsense` code has __NOT__ been ported to ros2 yet

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

The parameter files need to be in the directory where these commands are being run, the system to supply path for the param file is still a work in progress.
For LinkTrack:

```
ros2 launch nlink_parser_ros2 linktrack.launch.py
```

For LinkTrack AoA:

```
ros2 launch nlink_parser_ros2 linktrack_aoa.launch.py
```