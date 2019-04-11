# Chusei 3d webcam ros package


dependency:
```
sudo apt install uvcdyrctrl
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
![](coke.png)

run rqt reconfigure for disparity map's dynamic reconfiguration