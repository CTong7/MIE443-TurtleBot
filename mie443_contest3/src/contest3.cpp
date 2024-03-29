#include <header.h>
//include "/home/chris/mie443_ws/src/MIE443-TurtleBot/mie443_contest3/include/header.h"
#include <ros/package.h>
#include <imageTransporter.hpp>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

//-------------------LAUNCH INSTRUCTIONS FOR COMPUTER TESTING -----------------------

//STEP 1: LAUNCH THE WORLD FILE
//roslaunch turtlebot_gazebo turtlebot_world.launch

//STEP 2: LAUNCH NOISE PLAYING NODE
//rosrun sound_play soundplay_node.py

//STEP 3: RUN CONTEST 3 CODE
//rosrun mie443_contest3 contest3
 
//PLAY SOUND FILE BY ADDING .WAV FILE TO "SOUNDS FOLDER"

//-------------------------------------------------------------


//-------------------LAUNCH INSTRUCTIONS FOR REAL WORLD TESTING-----------------------

//STEP 1: INITIALIZE KOBUKI BASE
//roslaunch turtlebot_bringup minimal.launch

//STEP 2: LAUNCH NOISE PLAYING NODE
//rosrun sound_play soundplay_node.py

//STEP 3 : ENSURE PERSON IS STANDING 1 METER IN FRONT
//roslaunch turtlebot_follower follower.launch

//STEP 4: RUN CONTEST 3 CODE
//rosrun mie443_contest3 contest3
 
//PLAY SOUND FILE BY ADDING .WAV FILE TO "SOUNDS FOLDER"

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	//-----------------------ROS INITIALIZATION------------------------------------------------
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/"; // defining file path to .wav files
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

    // contest count down timer = 7 MINUTES
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	//--------DEFINE 2 IMAGE TRANSPORT CLASSES: 1RGB CLASS & 1 DEPTH CLASS --------------------
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	//-----------------------ROS INITIALIZATION-------------------------------------------------

	//PLAYS SOUND WAVE
	sc.playWave(path_to_sounds + "sound.wav"); // specify name of wave file

	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){
		ros::spinOnce(); // obtain new info from topics

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		}else if(world_state == 1){
			/*
			...
			...
			*/
		}
	}

	return 0;
}
