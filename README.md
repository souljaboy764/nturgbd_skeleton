# nturgbd_skeleton
This package implements a TF broadcaster for publishing the skeleton data from the NTU RGB-D Dataset. 

## Prerequisites
This codebase was developed and tested  on Ubuntu 18.04 with ROS melodic.

## Running the node
After building this package in your workspace using `catkin_make`, run the below command
```bash
roslaunch nturgbd_skeleton node.launch skeleton_filename:=<path to skeleton_file> camera_frame_id:=<your camera frame id>
```
If the `skeleton_filename` and `camera_frame_id` are omitted from the command, the default values in the launch file shall be used.

You can visualize the joints as 
- TFs in rviz or using the `view_frames` node from the tf package
- a MarkerArray on the topic `/nturgbd_skeleton_viz`
