# Chusei 3d webcam ros package


Dependency:
```
sudo apt install uvcdynctrl
```
if calibration needed:
> http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
```
rosrun camera_calibration cameracalibrator.py --approximate 0.1 --size 8x6 --square 0.027 --no-service-check right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left
```
normal stereo:
```
 roslaunch little_stereo_camera stereo.launch
```
with depth:
> http://wiki.ros.org/stereo_image_proc
```
ROS_NAMESPACE=stereo rosrun stereo_image_proctereo_image_proc
```
![coke](pics_for_readme/coke.png)

for disparity map's dynamic reconfiguration
```
rosrun rqt_reconfigure rqt_reconfigure
```
disparity map:
```
rosrun image_view disparity_view image:=/stereo/disparity
```
![disparity_map](pics_for_readme/disparity.png)

![3d_scene](pics_for_readme/stereo_long_distance.gif)

