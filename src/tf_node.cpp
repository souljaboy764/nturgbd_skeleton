/* Node to read skeleton data from the NTU RGB-D dataset and publish the tf */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <string>
#include <utility>
#include <iostream>
#include <fstream>

using namespace std;

enum {SPINEBASE, SPINEMID, NECK, HEAD, SHOULDERLEFT, ELBOWLEFT, WRISTLEFT, HANDLEFT, SHOULDERRIGHT, ELBOWRIGHT, WRISTRIGHT, HANDRIGHT, HIPLEFT, KNEELEFT, ANKLELEFT, FOOTLEFT, HIPRIGHT, KNEERIGHT, ANKLERIGHT, FOOTRIGHT, SPINESHOULDER, HANDTIPLEFT , THUMBLEFT, HANDTIPRIGHT, THUMBRIGHT};
vector<string> joint_names({"SPINEBASE", "SPINEMID", "NECK", "HEAD", "SHOULDERLEFT", "ELBOWLEFT", "WRISTLEFT", "HANDLEFT", "SHOULDERRIGHT", "ELBOWRIGHT", "WRISTRIGHT", "HANDRIGHT", "HIPLEFT", "KNEELEFT", "ANKLELEFT", "FOOTLEFT", "HIPRIGHT", "KNEERIGHT", "ANKLERIGHT", "FOOTRIGHT", "SPINESHOULDER", "HANDTIPLEFT", "THUMBLEFT", "HANDTIPRIGHT", "THUMBRIGHT"});

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nturgbd_skeleton_node");
	ros::NodeHandle nh;
	ros::Rate r(10);
	if(argc < 2)
	{
		ROS_INFO("Usage: rosrun nturgbd_skeleton nturgbd_skeleton_node <skeleton file path>");
		ros::shutdown();
		return -1;
	}

	string skeleton_filename(argv[1]);
	while(ros::ok())
	{
		ROS_INFO("Opening file %s", skeleton_filename);
		
		ifstream skeleton_file(skeleton_filename);
		if(!skeleton_file.is_open())
		{
			ROS_INFO("Unable to open file %s", skeleton_filename);
			ros::shutdown();
			return -1;
		}
		
		int frame_count;
		skeleton_file >> frame_count;
		if(frame_count==0)
		{
			ROS_INFO("No frames present");
			ros::shutdown();
			return -1;
		}


		
		static tf2_ros::TransformBroadcaster br;
		vector<geometry_msgs::TransformStamped> body_transforms(50); //25 per person and maximum 2 people in a frame
		
		for(int i=0;i<2;i++)
			for(int j=0;j<25;j++)
			{
				int index = i*25 + j;
				body_transforms[index].header.seq = 0;
				body_transforms[index].header.frame_id = "camera";
				body_transforms[index].child_frame_id = "nturgbd_skeleton_"+to_string(i)+"_"+joint_names[j];
			}
		for(int frame=0;frame<frame_count;frame++)
		{
			int body_count;
			skeleton_file>>body_count; //no. of observerd skeletons in current frame
			
			ros::Time t = ros::Time::now();
			for(int body_num=0;body_num<body_count;body_num++)
			{
				long int bodyID;
				skeleton_file >> bodyID;
				
				int clipedEdges, handLeftConfidence, handLeftState, handRightConfidence, handRightState, isResticted;
				skeleton_file >> clipedEdges >> handLeftConfidence >> handLeftState >> handRightConfidence >> handRightState >> isResticted;

				float leanX, leanY;
				skeleton_file >> leanX >> leanY;

				int bodyTrackingState;
				skeleton_file >> bodyTrackingState;

				int joint_count;
				skeleton_file >> joint_count;

				for(int joint_num=0; joint_num < joint_count; joint_num++)
				{
					float x, y, z, depthX, depthY, colorX, colorY, orientationW, orientationX, orientationY, orientationZ;
					skeleton_file >> x >> y >> z >> depthX >> depthY >> colorX >> colorY >> orientationW >> orientationX >> orientationY >> orientationZ;
					
					int jointTrackingState;
					skeleton_file >> jointTrackingState;

					if(bodyTrackingState and jointTrackingState)
					{
						ROS_INFO("TRACKING JOINT %d of SKELETON %d in FRAME %d",joint_num, body_num, frame);
						body_transforms[body_num*25 + joint_num].transform.translation.x = x;
						body_transforms[body_num*25 + joint_num].transform.translation.y = y;
						body_transforms[body_num*25 + joint_num].transform.translation.z = z;

						body_transforms[body_num*25 + joint_num].transform.rotation.x = orientationX;
						body_transforms[body_num*25 + joint_num].transform.rotation.y = orientationY;
						body_transforms[body_num*25 + joint_num].transform.rotation.z = orientationZ;
						body_transforms[body_num*25 + joint_num].transform.rotation.w = orientationW;

						body_transforms[body_num*25 + joint_num].header.stamp = t;
					}
				}
			}
			br.sendTransform(body_transforms);
			r.sleep();
		}
		skeleton_file.close();
	}

	return 0;
}
