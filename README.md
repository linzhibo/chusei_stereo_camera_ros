# Chusei 3d webcam ros package


dependency:
```
sudo apt install uvcdyrctrl
```
if calibration needed:
> http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration

normal stereo:
```
 roslaunch little_stereo_camera stereo.launch
```
with depth:
> http://wiki.ros.org/stereo_image_proc
```
ROS_NAMESPACE=stereo rosrun stereo_image_proctereo_image_proc
```
<img src="pics_for_readme/coke.png" alt="drawing" width="300"/>

>run rqt reconfigure for disparity map's dynamic reconfiguration

disparity map:
```
rosrun image_view disparity_view image:=/stereo/disparity
```
<img src="pics_for_readme/disparity.png" alt="drawing" width="500"/>
3d scene
<img src="pics_for_readme/stereo_long_distance.gif" alt="drawing" width="500"/>
