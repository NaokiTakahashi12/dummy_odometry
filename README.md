# dummy_odometry

This package is a simplified simulation package for coordinate transformation between the `odom` frame and the `base_link` frame.
It utilizes velocity commands, either `/cmd_vel` or `/cmd_vel_stamped`, for the simplified odometry simulation. (The topic timestamp can be modified using parameters.)

## Usage

```bash
$ ros2 run dummy_odometry dummy_odometry_node
```
Upon executing the above command, the node will wait until it subscribes to the `/dummy_odometry_node/cmd_vel` topic.
Once it receives velocity commands, it will publish the transform from the `odom` frame to the `base_link` frame, as well as the `/dummy_odometry_node/odom` topic.

## Dependencies

1. rclcpp
2. generate_parameter_library
3. tf2_ros
4. geometry_msgs
5. nav_msgs

## Tested environment

This package has been tested in the following environment:
+ ROS 2 Humble
