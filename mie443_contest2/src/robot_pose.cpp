#include <robot_pose.h>
#include <tf/transform_datatypes.h>

//Creating a RobotPose Object with type RobotPose class
RobotPose::RobotPose(float x, float y, float phi) {
	this->x = x; //this-> is a pointer to x, differentiate the class name and the object name, it helps access tthe class properties
	this->y = y;
	this->phi = phi;
}

//Creates vraiables x,y and phi that we can access
void RobotPose::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
	phi = tf::getYaw(msg.pose.pose.orientation);
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
}
