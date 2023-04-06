#include <header.h>
//include "/home/chris/mie443_ws/src/MIE443-TurtleBot/mie443_contest3/include/header.h"
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/BumperEvent.h>


#include <iostream>
#include <chrono>
#include <thread>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
uint8_t wheel[2]={kobuki_msgs::WheelDropEvent::RAISED, kobuki_msgs::WheelDropEvent::RAISED};

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
	// How to detect when we have lost track of person?
	// ROS_INFO("TurtleBot has been follow");

}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
    //ROS_INFO("Bumper #%lf pressed.", bumper);
	ROS_INFO("TurtleBot has been bumped");
}


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
	ros::Subscriber wheel_drop = nh.subscribe("mobile_base/events/wheel_drop", 10, &wheel_drop_CB);

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
	//sc.playWave(path_to_sounds + "sound.wav"); // specify name of wave file

	ros::Duration(0.5).sleep();

	// We only want to execute fear once ever
	bool afraid_exit = false;

	while(ros::ok() && secondsElapsed <= 480){
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		vel_pub.publish(follow_cmd);


		bool wheel_drop_triggered = false; //wheel_drop_triggered = 0

		for (uint32_t w_idx = 0; w_idx < 2; w_idx++){

			// Check if either the LEFT (0) or Right (1) wheels have been lowered
			// Bitwise OR operation; as long as one of the wheels returns 0, wheel_drop_triggered will return false
			wheel_drop_triggered |= (wheel[w_idx] == kobuki_msgs::WheelDropEvent::DROPPED);
			// True when wheels are tucked in, false when one wheel is dropped

		}

	

		// int i;
		// for (i = 0; i < 3;i++){
		// 	ros::spinOnce(); // obtain new info from topics
		// 	ros::Duration(0.01).sleep();
		// 	//Always publish follow cmds.
		// 	vel_pub.publish(follow_cmd);
		// }
		

		/* Set world state based on sensor info
		world_state == 0 -> 
		world_state == 1 -> Afraid emotion
		world_state == 2 -> 
		world_state == 3 -> 
		world_state == 4 -> 
		
		*/
	// Manual Setting
		//world_state = 1;

		// Trigger afraid if loses track of person, and time > 5 seconds
		ROS_INFO("follow cmd: %f", follow_cmd.linear.x);
		ROS_INFO("Seconds ealpseds: %lu", secondsElapsed);
		
		
		if (follow_cmd.linear.x < 0.1 && follow_cmd.linear.x > -0.1 && secondsElapsed > 5 && !afraid_exit){ // If afraid_exit is true, never execute.
			world_state = 1;
		} else if (wheel_drop_triggered) {
			ROS_INFO("Triggered");
			// If one of the wheels has dropped, then enter into world_state 3
			world_state = 3;
		}
		else {
			world_state = 0;
		}

		// Executes code based on World State
		if(world_state == 0){
			//fill with your code
			// vel_pub.publish(vel);
			//sc.playWave(path_to_sounds + "sound.wav"); 
			ROS_INFO("World State: %i", world_state);
			vel_pub.publish(follow_cmd);

		} else if(world_state == 1){
			// Afraid Code
			// Show video/image/gif
			
			
			ROS_INFO("World State: %i", world_state);
			// Play sound - Better for it to be unambiguous than accurate
			sc.playWave(path_to_sounds + "afraid.wav"); // specify name of wave file

			//Move around as if afraid.
			/*
			- Spin around continuosly in a loop
			- Fast, slow down, then spin the other way fast.
			- Exit condition: ???
			*/

			// Spin ccw for 5 seconds
			// auto scared_timer_start = std::chrono::steady_clock::now();
    		// auto scared_timer_end = scared_timer_start + std::chrono::seconds(5);
			angular = 2.0;
			std::chrono::time_point<std::chrono::system_clock> scared_timer_start;
    		scared_timer_start = std::chrono::system_clock::now();
			uint64_t scared_duration = 0;
			uint64_t scared_target_duration_long = 1;
			uint64_t scared_target_duration_short = 1;

			
			while (!afraid_exit){
				scared_timer_start = std::chrono::system_clock::now();
				scared_duration = 0;

				while (scared_duration < scared_target_duration_long) {

					ROS_INFO("Hello");
					vel.angular.z = angular;
					vel_pub.publish(vel);
					scared_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-scared_timer_start).count();
					std::this_thread::sleep_for(std::chrono::milliseconds(100)); // pause for 100 ms

				}

				scared_timer_start = std::chrono::system_clock::now();
				scared_duration = 0;
				while (scared_duration < scared_target_duration_short) {
					vel.angular.z = 0.0;
					std::this_thread::sleep_for(std::chrono::milliseconds(100)); // pause for 100 ms
					vel_pub.publish(vel);
					scared_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-scared_timer_start).count();

				}		

				scared_timer_start = std::chrono::system_clock::now();
				scared_duration = 0;
				while (scared_duration < scared_target_duration_long) {
					vel.angular.z = -angular;
					std::this_thread::sleep_for(std::chrono::milliseconds(100)); // pause for 100 ms
					vel_pub.publish(vel);
					scared_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-scared_timer_start).count();

				}	

				scared_timer_start = std::chrono::system_clock::now();
				scared_duration = 0;
				while (scared_duration < scared_target_duration_short) {
					vel.angular.z = 0.0;
					std::this_thread::sleep_for(std::chrono::milliseconds(100)); // pause for 100 ms
					vel_pub.publish(vel);
					scared_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-scared_timer_start).count();

				}	

				
				afraid_exit = true;


			}  
			
		} else if(world_state == 3) {
			ROS_INFO("World State 3");
			while (wheel_drop_triggered){
				
				// While the TurtleBot is raise, play sound
				sc.playWave(path_to_sounds + "excited.wav");
				std::this_thread::sleep_for(std::chrono::milliseconds(6000)); // pause for 100 ms

				// Show image of Homer cheering	on the laptop screen	

			}

		}
    		

		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

	}

	return 0;
}
