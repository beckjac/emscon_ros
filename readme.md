# EmsconROS
This is a ROS package designed to interface with Leica laser trackers using the Emscon API.

## Building
Just clone into your catkin workspace and use `catkin_make`. Boost is used for sockets, but that should be included with your installation of ROS, and a copy of the Emscon API is included, so no external dependencies should be required.

## Usage
Run `roslaunch emscon_ros test_emscon.launch`, and once the initialization process has completed, laser data should be published on the topic `/laser_measurements` as `PointStamped` messages.

## To Do
All options (server IP, port number, measurement rate, reflector name, etc.) are currently hard coded in `main.cpp`. At some point these should be made into parameters that can be set via the launch file.

There is currently no error recovery behavior for a broken laser beam. If this happens, the node must be restarted. This should obviously be fixed.
