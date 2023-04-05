#include <nav_msgs/GetPlan.h>
#include <vector>
#include <robot_pose.h>
#include <cmath>
#include <math.h>
#include <geometry_msgs/PoseStamped.h> // getplan takes gemoetry messages as an input
#include <ros/service_client.h>
#include <numbers>


class targetclass {
	public:
		vector<float> target_pose(RobotPose robotPose, vector<float> target_object);
};