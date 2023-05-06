# CameraMount
The camera mount has 2 DoF and can be controlled using serial communication. The camera on the mount is a RealSense camera.The camera mount can be controlled via two methods. The first method is via keyboard control, and the second method is where the rotation of the Oculus Quest 2 is mapped to the camera mount. This repo contains all the code for controlling the motion of the camera mount. To get the camera feed you must use the [realsense_gopher](https://github.com/geekblack22/realsense_gopher) repository.

## Running the camera mount
### ROS
#### ROS Unity TCP
'roslaunch ros_tcp_endpoint endpoint.launch'
#### Oculus Control
'rosrun mobile_camera control_head.py'
#### Camera Feed
'roslaunch realsense_gopher real_gopher.launch  segment:=false'
#### Keyboard Controll
call the keyboard_control function of the CameraMount Class in the control_mount.py file

### Unity
Run the Input Control - servo unity project




