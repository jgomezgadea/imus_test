# imus_test
ROS package to test the precision of the IMUs

## 1 imus_test
Starts dynamixel motors and all IMU with the same orientation to check IMU's entropy.

### 1.1 Subscribed Topics
* /joint_1/command (std_msgs/Float64)
  Position control of the horizontal rotation motor (rad)

* /joint_2/command (std_msgs/Float64)
  Position control of the vertical rotation motor (rad)
   
### 1.2 Published Topics
* /px_2_4_8/yaw (std_msgs/Float64)
  Orientation of the IMU with an applied filter (rad)

* /px_cube/yaw (std_msgs/Float64)
  Orientation of the IMU with an applied filter (rad)

* /myahrs/yaw (std_msgs/Float64)
  Orientation of the IMU with an applied filter (rad)

* /phidgets/yaw (std_msgs/Float64)
  Orientation of the IMU with an applied filter (rad)

* /phidgets/other_yaw (std_msgs/Float64)
  Orientation of the IMU with an applied filter (rad)

* /phidgets/other_yaw (std_msgs/Float64)
  Orientation of the IMU with an applied filter (rad)

* /arm_controller/state/actual/positions[0] (control_msgs/FollowJointTrajectoryFeedback)
  Position of the horizontal rotation motor (rad)

* /arm_controller/state/actual/positions[1] (control_msgs/FollowJointTrajectoryFeedback)
  Position of the vertical rotation motor (rad)

### 1.3 Bringup
* $ roslaunch imus_test imus_test.launch

## 2 imus_test_msgs
Messages for imus_test pkg

## 3 myahrs_driver
Driver for myahrs IMU

## 4 phidgets_driver
Driver for phidgets IMU

## 5 widowx_arm
Driver for dynamixel motors

