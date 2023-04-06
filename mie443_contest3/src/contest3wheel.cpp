#include <header.h>
//include "/home/chris/mie443_ws/src/MIE443-TurtleBot/mie443_contest3/include/header.h"
#include <ros/package.h>
//#include "/home/andrew/test/src/MIE443-TurtleBot/mie443_contest3/include/imageTransporter.hpp"
#include <imageTransporter.hpp>
#include <kobuki_msgs/WheelDropEvent.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;

// Initialize vector of size 2 with all values = raised
uint8_t wheel[2]={kobuki_msgs::WheelDropEvent::RAISED, kobuki_msgs::WheelDropEvent::RAISED};

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const geometry_msgs::Twist msg){
    //Fill with code
}

// Define wheel drop callback function

void wheel_drop_CB(const kobuki_msgs::WheelDropEvent::ConstPtr& msg){
	// A WheelDropEvent has 2 properties .wheel (LEFT=0,RIGHT=1) and .state (RAISED=0,DROPPED=1)
	wheel[msg->wheel] = msg->state; // -> operator indicates that msg is a pointer to the WheelDropEvent message
	ROS_INFO("TurtleBot has been lifted");
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

		bool wheel_drop_triggered = false; //wheel_drop_triggered = 0

		for (uint32_t w_idx = 0; w_idx < 2; w_idx++){

			// Check if either the LEFT (0) or Right (1) wheels have been lowered
			// Bitwise OR operation; as long as one of the wheels returns 0, wheel_drop_triggered will return false
			wheel_drop_triggered |= (wheel[w_idx] == kobuki_msgs::WheelDropEvent::RAISED);

		}

		if(wheel_drop_triggered){
			
			// If one of the wheels has dropped, then enter into world_state 3
			world_state = 3;

		}

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);

		//}else if(world_state == 1){
			/*
			...
			...
			*/
		//}else if(world_state == 2){
			/*
			...
			...
			*/			
		}else if(world_state == 3){
			
			while (wheel_drop_triggered){
				
				// While the TurtleBot is raise, play sound
				sc.playWave(path_to_sounds + "excited.wav");

				// Show image of Homer cheering	on the laptop screen	

			}

		}
		}

	return 0;

}


