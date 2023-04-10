#include <header.h>
//include "/home/chris/mie443_ws/src/MIE443-TurtleBot/mie443_contest3/include/header.h"
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <kobuki_msgs/BumperEvent.h>

using namespace std;

geometry_msgs::Twist follow_cmd;
int world_state;
//------INITIALIZE VECTOR OF SIZE 3 WITH ALL VALUES = RELEASED,---------------------------
uint8_t bumper[3]={kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED};

void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

//callback function loads message from topic into a global variable so that it can be accessed
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	//a bumperEvent object has 2 properties .state (RELEASED, PRESSED) and .bumper (0,1,2)
	bumper[msg->bumper]=msg->state; // -> operator indicates that msg is a pointer to the BumperEvent message
	ROS_INFO("bumper is hit");
    //Fill with code
}

//anger when it hits obstacle
//fear when it loses sight
//sadness

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
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1); // it should take input from teleop

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumpersub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); //subscribe to bumper topic
	//& in front of &bumperCallback gest teh addres of the funciton

    // contest count down timer = 7 MINUTES
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	//--------DEFINE 2 IMAGE TRANSPORT CLASSES: 1RGB CLASS & 1 DEPTH CLASS --------------------
	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int world_state = 0;

	float angular = 0.2; // angular and linear are doubles
	float linear = 0.0;

	geometry_msgs::Twist vel; //gemoetry_msgs is a float64, create a vel object (indexed through vel.linear.x or vel.angular.x)
	vel.angular.z = angular;
	vel.linear.x = linear;

	//-----------------------ROS INITIALIZATION-------------------------------------------------

	

	ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){
		ros::spinOnce(); // obtain new info from topics




		//-----------------------LOADING BUMPER INFO--------------------------
		bool any_bumper_pressed=false;

		for (uint32_t b_idx=0;b_idx<3; b_idx++){

			any_bumper_pressed |= (bumper[b_idx]==kobuki_msgs::BumperEvent::PRESSED); //bitwise or, as long as one of the bumpers returns 1, any_bumper_pressed will return false
			//can i just do bumper[b_idx].state?? so much easier to understand
			// how can you even multiply a boolean and a 0/1???
		}
		//-----------------------LOADING BUMPER INFO--------------------------
		if(any_bumper_pressed){
			world_state=0;
		}
		else{
			world_state=1;
		}

		if(world_state==0){
			//fill with your code
			//vel_pub.publish(vel);

			//-----------------PLAYS ANGRY SOUND WAVE-----------------------------
			sc.playWave(path_to_sounds + "goku.wav"); // specify name of wave file

			//-----------------SHOW PNG--------------------------------------------

			cv::Mat image =cv::imread("/home/chris/Desktop/test.png");
			///home/chris/mie443_ws/src/MIE443-TurtleBot/mie443_contest3/src/angry.png
			cv::imshow("image", image);
			


			for (uint32_t i=0;i<3; i++){

				//reverse by 20cm
				angular=0.0;
				linear=-0.5; //meters per second

				vel.angular.z=angular;
				vel.linear.x=linear;
				vel_pub.publish(vel);

				//Sleep so it can travel that far
				ros::Duration(0.5).sleep();//unit: seconds

				//add a time delay to let it travel that distance

				// drive forward 20cm
				// if we actually want slamming action we have to make it drive fowrard until bumper is hit again
				angular=0.0;
				linear=0.5; // meters per second

				vel.angular.z=angular;
				vel.linear.x=linear;
				vel_pub.publish(vel); //name of publisher: vel_pub
				ros::Duration(0.5).sleep();//unit: seconds

				

			}
		}

		else if (world_state==1){
			
			

		}

			//add a delay

			// movebackwards
			// angular=0.0;
			// linear=1.0;

			// vel.angular.z=angular;
			// vel.linear.x=linear;
			// vel_pub.publish(vel); //name of publisher: vel_pub

			//vel_pub.publish(follow_cmd);
		
		// }else if(world_state == 1){
		// 	/*
		// 	...
		// 	...
		// 	*/
		// }
	}

	return 0;
}
