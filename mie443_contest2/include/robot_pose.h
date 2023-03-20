#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
//Creating a RobotPose Class type, classes are types not variables
//This class has 3 public variables: x,y and phi
//It also ahas 2 public functions: RobotPose and poseCallBack
class RobotPose {
	public:
		float x;
		float y;
		float phi;
	public:
		RobotPose(float x, float y, float phi);
		void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};
