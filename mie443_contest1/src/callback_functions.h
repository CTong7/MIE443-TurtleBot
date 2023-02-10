#ifndef DEPENDENCIES
#define DEPENDENCIES

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <iterator>
#include <vector>

//#include "robot_functions.h"
//TODO: add #ifndef

// Added for the ODOM demo
#include <nav_msgs/Odometry.h> 
#include <tf/transform_datatypes.h> 

// Added for BUMPER demo
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

// Added for LASER demo
float minLaserDist = std::numeric_limits<float>::infinity();
int32_t nLasers=0, desiredNLasers=0, desiredAngle=60, laser_array_size = 639;
int32_t laser_min_index=320; // Half of laser scan array size
float test_dist=0;
std::vector<float> laser_scan_array;
int open_space_count_right =0;
int open_space_count_left =0;
int closed_space_count=0;

float posX = 0.0, posY = 0.0, yaw = 0.0;
float angular_vel_to_be_published = 0.0;
float linear_vel_to_be_published = 0.0;


#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr&);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr&);
void odomCallback (const nav_msgs::Odometry::ConstPtr&);
#endif