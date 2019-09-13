# NTU RGB-D Skeleton TF Publisher
This package implements a TF broadcaster for publishing the skeleton data from the NTU RGB-D Dataset. 

## Prerequisites
This codebase was developed and tested  on Ubuntu 18.04 with ROS melodic.

## Running the node
After building this package in your workspace using `catkin_make`, run the below command
```
rosrun nturgbd_skeleton node
```
You can visualize the TFs in rviz or use the `view_frames` node from the tf package.

## TODO
- Add visualization markers for the joints
- Add option to publish as NuiTrack joints instead of kinect joints