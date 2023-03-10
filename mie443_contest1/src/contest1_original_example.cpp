#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <cmath>
#include <chrono>

// Added for the ODOM demo
#include <nav_msgs/Odometry.h> 
#include <tf/transform_datatypes.h> 
float posX = 0.0, posY = 0.0, yaw = 0.0;
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)


// Added for BUMPER demo
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};


// Added global variables for LASER demo
float minLaserDist=std::numeric_limits<float>::infinity(); //setting it to positive infinity
int32_t nLasers=0,desiredNLasers=0,desiredAngle=5;


// SECOND EXAMPLE: Bumper
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
    // nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    nLasers=(msg->angle_max-msg->angle_min)/msg->angle_increment;
    desiredNLasers=DEG2RAD(desiredAngle)/msg->angle_increment;
    ROS_INFO("size of the laser scan array %i and size of offset %i",nLasers,desiredNLasers);

    // minLaserDist = std::numeric_limits<float>::infinity();
    // nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    // if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
    //     for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
    //         minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
    //     }
    // }
    // else {
    //     for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
    //         minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
    //     }
    // }

}

// FIRST EXAMPLE: Odom
void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    // // //fill with your code
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener"); //can change the node name to anything
    ros::NodeHandle nh;

    // 2 subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

    // 1 publisher
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();


    //     // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
    //     // Check if any of the bumpers were pressed.
    //     bool any_bumper_pressed = false;
    //     for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
    //         any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    //     }    

    //     if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
    //         angular = 0.0;
    //         linear = 0.2;
    //     }
    //     else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5) {
    //         angular = M_PI / 6;
    //         linear = 0.0;
    //     }
    //     else if (minLaserDist > 1. && !any_bumper_pressed) {
    //         linear = 0.1;
    //         if (yaw < 17 / 36 * M_PI || posX > 0.6) {
    //             angular = M_PI / 12.;
    //         }
    //         else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
    //             angular = -M_PI / 12.;
    //         }
    //         else {
    //             angular = 0;
    //         }
    //     }
    //     else {
    //         angular = 0.0;
    //         linear = 0.0;
    //     }

    //     vel.angular.z = angular;
    //     vel.linear.x = linear;
    //     vel_pub.publish(vel);

    //     // The last thing to do is to update the timer.
    //     secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    //     loop_rate.sleep();

    
    }

    // return 0;
}




        // //
        // // Check if any of the bumpers were pressed.
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        // any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }
        // //
        // // Control logic after bumpers are being pressed.
        // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
        //     angular = 0.0;
        //     linear = 0.2;
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }
        // else {
        //     angular = 0.0;
        //     linear = 0.0;
        // }
        // vel.angular.z = angular;
        // vel.linear.x = linear;        // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        
        
        // // Check if any of the bumpers were pressed.
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        //     any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }    

        // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
        //     angular = 0.0;
        //     linear = 0.2;
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5) {
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }
        // else if (minLaserDist > 1. && !any_bumper_pressed) {
        //     linear = 0.1;
        //     if (yaw < 17 / 36 * M_PI || posX > 0.6) {
        //         angular = M_PI / 12.;
        //     }
        //     else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
        //         angular = -M_PI / 12.;
        //     }
        //     else {
        //         angular = 0;
        //     }
        // }
        // else {
        //     angular = 0.0;
        //     linear = 0.0;
        // }


        // // Check if any of the bumpers were pressed.
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        // any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }
        // //
        // // Control logic after bumpers are being pressed.
        // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) {
        //     angular = 0.0;
        //     linear = 0.2;
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) {
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }
        // else {
        //     angular = 0.0;
        //     linear = 0.0;
        // }
        // vel.angular.z = angular;
        // vel.linear.x = linear;
        // vel_pub.publish(vel);




        // SCAN EXAMPLE
        // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        // // Check if any of the bumpers were pressed.
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        //     any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }    

        // if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed && minLaserDist > 0.7) {
        //     angular = 0.0;
        //     linear = 0.2;
        // }
        // else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed && minLaserDist > 0.5) {
        //     angular = M_PI / 6;
        //     linear = 0.0;
        // }
        // else if (minLaserDist > 1. && !any_bumper_pressed) {
        //     linear = 0.1;
        //     if (yaw < 17 / 36 * M_PI || posX > 0.6) {
        //         angular = M_PI / 12.;
        //     }
        //     else if (yaw < 19 / 36 * M_PI || posX < 0.4) {
        //         angular = -M_PI / 12.;
        //     }
        //     else {
        //         angular = 0;
        //     }
        // }
        // else {
        //     angular = 0.0;
        //     linear = 0.0;
        // }