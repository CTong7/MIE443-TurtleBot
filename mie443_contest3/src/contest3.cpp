#include <header.h>
//include "/home/chris/mie443_ws/src/MIE443-TurtleBot/mie443_contest3/include/header.h"
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/WheelDropEvent.h>
#include <kobuki_msgs/BumperEvent.h>
#include "opencv2/opencv.hpp"


#include <iostream>
#include <chrono>
#include <thread>

using namespace std;
using namespace cv;

geometry_msgs::Twist follow_cmd;
int world_state;
uint8_t wheel[2]={kobuki_msgs::WheelDropEvent::RAISED, kobuki_msgs::WheelDropEvent::RAISED};

uint8_t bumper_state[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
	// How to detect when we have lost track of person?
	// ROS_INFO("TurtleBot has been follow");

}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper_state[msg->bumper] = msg->state;
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

	cv::Mat afraid_img, excited_img, angry_img, sad_img;
	string path_to_imgs = ros::package::getPath("mie443_contest3") + "/images/";
	afraid_img = cv::imread(path_to_imgs + "afraid.png");

	excited_img = cv::imread(path_to_imgs + "excited.png");
	sad_img = cv::imread(path_to_imgs + "sad-dog.jpg");
	angry_img = cv::imread(path_to_imgs + "angry.png");

	// // Testing
	// imshow("1",afraid_img);
	// waitKey(0);
	// imshow("1",excited_img);
	// waitKey(0);
	// imshow("1",sad_img);
	// waitKey(0);
	// imshow("1",angry_img);
	// waitKey(0);

	std::chrono::time_point<std::chrono::system_clock> scared_timer_start, excited_timer_start, sound_timer_start, image_timer;
	scared_timer_start = std::chrono::system_clock::now();
	uint64_t scared_duration = 0;
	float scared_target_duration_long = 1;
	float scared_target_duration_short = 0.25;

	uint64_t excited_duration = 0;
	float excited_target_duration_long = 1;
	float excited_target_duration_short = 0.25;

	uint64_t sound_duration = 0;

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
	bool afraid_exit_lock = false;
	bool sad_follow_lock = false;
	bool has_just_exited_afraid = false;
	bool prompt_for_name = false;

	bool sound_initial_play = true;

	while(ros::ok() && secondsElapsed <= 480){
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		vel_pub.publish(follow_cmd);

		bool wheel_drop_triggered = false; //wheel_drop_triggered = 0
		string user_name;

		for (uint32_t w_idx = 0; w_idx < 2; w_idx++){

			// Check if either the LEFT (0) or Right (1) wheels have been lowered
			// Bitwise OR operation; as long as one of the wheels returns 0, wheel_drop_triggered will return false
			wheel_drop_triggered |= (wheel[w_idx] == kobuki_msgs::WheelDropEvent::DROPPED);
			// True when wheels are tucked in, false when one wheel is dropped

		}

		bool any_bumper_pressed=false;

		for (uint32_t b_idx=0; b_idx<3; b_idx++){
			any_bumper_pressed |= (bumper_state[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //bitwise or, as long as one of the bumpers returns 1, any_bumper_pressed will return false
			cout<< "bumper for loop. Value: " << any_bumper_pressed << endl;
			//can i just do bumper[b_idx].state?? so much easier to understand
			// how can you even multiply a boolean and a 0/1???
		}
		
		/* Set world state based on sensor info
		world_state == 0 -> 
		world_state == 1 -> Afraid emotion
		world_state == 2 -> 
		world_state == 3 -> 
		world_state == 4 -> 
		
		*/
		
		ROS_INFO("follow cmd: %f", follow_cmd.linear.x);
		ROS_INFO("Seconds ealpseds: %lu", secondsElapsed);
		
		//Check sensors and set world_state
		if (!afraid_exit_lock && !any_bumper_pressed && !sad_follow_lock) { // If afraid_exit_lock is true, never execute.
			if ((follow_cmd.linear.x < 0.05 && follow_cmd.linear.x > -0.05 && secondsElapsed > 5)){
				ROS_INFO("World State Afraid Update 1");
				world_state = 1; // scared
			}

		} else if (any_bumper_pressed && !sad_follow_lock){
			ROS_INFO("World State Angry Update 2");
			world_state = 2; // angry
		}
		
		else if (wheel_drop_triggered && !prompt_for_name) {
			
			destroyAllWindows(); 
			cout << "Wheel Drop Prompt Name" <<endl;
			sad_follow_lock = false;
			prompt_for_name = true;
			continue;

		} else if (has_just_exited_afraid && !sad_follow_lock){
			cout << "Has Just Exited Afraid" <<endl;
			if (follow_cmd.linear.x > 0.1 || follow_cmd.linear.x < -0.1){
				prompt_for_name = true;
				has_just_exited_afraid = false;
				continue;
			}
			else {
				world_state = 0;
				sc.playWave(path_to_sounds + "Where are you.wav"); // specify name of wave file

				cout << "Waiting to find Person" <<endl;
			}

		} else if (prompt_for_name){

			cout << "Please enter your Name: " <<endl;
    		getline(cin, user_name);

			if (user_name == "Andrew"){
				ROS_INFO("YAY I FOUND YOU AGAIN!");
				world_state = 3; // excited
				prompt_for_name = false;

			}
			else {
				sc.playWave(path_to_sounds + "You aren't my owner.wav"); // specify name of wave file

				ROS_INFO("You are not my owner ... ");
				world_state = 4; // Sad
				sad_follow_lock = true;
				prompt_for_name = false;
			}
		}
		else {
			world_state = 0;
			
		}

		if (sad_follow_lock){
			world_state=4;
		}
		//////////////////////// Executes code based on World State ///////////////////////
		if(world_state == 0){
			//fill with your code
			// vel_pub.publish(vel);
			//sc.playWave(path_to_sounds + "sound.wav"); 
			ROS_INFO("World State: %i", world_state);

			//follow_cmd.linear.x = 0.3;
			vel_pub.publish(follow_cmd);

		} else if(world_state == 1){
			// Afraid Code
			// Show video/image/gif
			
			
			ROS_INFO("World State: %i", world_state);
			
			//Move around as if afraid.
			/*
			- Spin around continuosly in a loop
			- Fast, slow down, then spin the other way fast.
			- Exit condition: ???
			*/

			// Spin ccw for 5 seconds
			// auto scared_timer_start = std::chrono::steady_clock::now();
    		// auto scared_timer_end = scared_timer_start + std::chrono::seconds(5);
			angular = 2.5;
			

			int counter = 0;
			cv::namedWindow("AHHHH", cv::WINDOW_NORMAL);
			cv::setWindowProperty("AHHHH", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			cv::imshow("AHHHH",afraid_img);


			// // Play sound twice
			// while (counter < 2){
			// Play sound - Better for it to be unambiguous than accurate
			sc.playWave(path_to_sounds + "afraid.wav"); // specify name of wave file
			waitKey(1);

			scared_timer_start = std::chrono::system_clock::now();
			scared_duration = 0;

			for (int i = 0; i <2; i++){
				while (scared_duration < scared_target_duration_long) {
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
			}
					
				// counter ++;

			// }  

			has_just_exited_afraid = true;
			afraid_exit_lock = true;
			cout << "Afraid Exit Lock" << afraid_exit_lock <<endl;
			destroyAllWindows();
			
		} else if (world_state == 2) {
			ROS_INFO("World State 2");
			//fill with your code
			//vel_pub.publish(vel);

			//-----------------PLAYS ANGRY SOUND WAVE-----------------------------
			sc.playWave(path_to_sounds + "angry-scream.wav"); // specify name of wave file

			//-----------------SHOW PNG--------------------------------------------

			cv::namedWindow("ANGRY", cv::WINDOW_NORMAL);
			cv::setWindowProperty("ANGRY", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			
			for (uint32_t i=0;i<6; i++){
				cv::imshow("ANGRY",angry_img);
				waitKey(1);

				//reverse by 20cm
				angular=0.0;
				linear=-0.2; //meters per second

				vel.angular.z=angular;
				vel.linear.x=linear;
				vel_pub.publish(vel);

				//Sleep so it can travel that far
				ros::Duration(0.5).sleep();//unit: seconds

				//add a time delay to let it travel that distance

				// drive forward 20cm
				// if we actually want slamming action we have to make it drive fowrard until bumper is hit again
				angular=0.0;
				linear=0.2; // meters per second

				vel.angular.z=angular;
				vel.linear.x=linear;
				vel_pub.publish(vel); //name of publisher: vel_pub
				ros::Duration(0.5).sleep();//unit: seconds

			}

			// Exits Anger Emotion
			destroyAllWindows();
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			world_state=0;
		}
		
		
		else if(world_state == 3) {
			ROS_INFO("World State 3");

			sc.playWave(path_to_sounds + "excited.wav");
			// Move in a small circle
			excited_timer_start = std::chrono::system_clock::now();
			excited_duration = 0;
			cv::namedWindow("Yay!", cv::WINDOW_NORMAL);
			cv::setWindowProperty("Yay!", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			while (excited_duration < 5) {
				
				imshow("Yay!", excited_img);
				waitKey(1);
				vel.angular.z = 2.0;
				vel.linear.x = 0.4;
				std::this_thread::sleep_for(std::chrono::milliseconds(100)); // pause for 100 ms
				vel_pub.publish(vel);
				excited_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-excited_timer_start).count();

			}	

			destroyAllWindows();


		}
		else if (world_state == 4) {

			ROS_INFO("World State 4");
			sad_follow_lock = true; // Make it so that sadness is always triggered until robot is picked up
			
			if (sound_initial_play){
				sc.playWave(path_to_sounds + "sad.wav");
				sound_initial_play = false;
				sound_timer_start = std::chrono::system_clock::now();
				sound_duration = 0;

			}

			follow_cmd.linear.x *= 0.5; // use this for sadness 
			vel_pub.publish(follow_cmd);
			cv::namedWindow("wahhh ...", cv::WINDOW_NORMAL);
			cv::setWindowProperty("wahhh ...", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
			imshow("wahhh ...", sad_img);
			waitKey(1);

			if(sound_duration > 103){
				sound_initial_play = true;

			}
			
			sound_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-sound_timer_start).count();
			prompt_for_name = false;
			world_state = 4;
		}


		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();

	}

	return 0;
}
