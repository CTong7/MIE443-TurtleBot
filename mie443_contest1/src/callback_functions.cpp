// #ifndef CALLBACKS // Do I need to do this? I have #ifndef in callback_functions.h already
// #define CALLBACKS // If i have function prototypes in a header file, do I still need to ifndef the implementations?
// Maybe I don't, but I am a little suspicious of my #include callback_functions.cpp line in contest1.cpp
#include "callback_functions.h"
// TODO: Use #ifndef

//int index_of_smallest_element(float arr[],)

// SECOND EXAMPLE: Bumper
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
    ROS_INFO("Bumper #%lf pressed.", bumper);
}

// This thing is dumber than I thought.
// The camera cannot do feature detection and identify obstacles it sees.
// It can only sense that it has detected something.
// HOwever, it can still tell you how far away it is and in which direction it sees it.
// But you can't see what that something is.

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
    // nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    minLaserDist = std::numeric_limits<float>::infinity();
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    // Checks if desiredAngle is less than max range of the laser scanner.
    // If true, cut out unnecessary laser scanner readings
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            if (!std::isinf(msg->ranges[laser_idx])){
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            }
        }

        // Copy the msg->ranges array into global variable
        // int arr_size = nLasers;
        // laser_scan_array.resize(arr_size);

        // std::copy(std::begin(msg->ranges), std::end(msg->ranges), std::back_inserter(laser_scan_array));
    }
    else {
        int mindex = 0;
        open_space_count_left=0;
        open_space_count_right =0;
        closed_space_count=0;

        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            if (msg->ranges[laser_idx] > 1.5){
                if (laser_idx < nLasers/3){
                    open_space_count_left++;
                }

                if (laser_idx > 2*nLasers/3){
                    open_space_count_right++;

                }
            }
            if (!std::isinf(msg->ranges[laser_idx])){
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
                
                if (msg->ranges[laser_idx] < msg->ranges[mindex]){
                    mindex = laser_idx;
                }
                // minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            }



        }
        laser_min_index = mindex;

        // Copy the msg->ranges array into global variable
        // int arr_size = nLasers;
        // laser_scan_array.resize(arr_size);

        // std::copy(std::begin(msg->ranges), std::end(msg->ranges), std::back_inserter(laser_scan_array));


    }

    // Based on tests, range_min is just the lowest sensible distance. Not the minimum distances in the laser distance array.
    // If you want to minimum distance, we must search msg->ranges[] ourself.
    // Can we set range_min ourselves? Can we make it lower so the camera senses lower distances? 
    ROS_INFO("Min Laser Dist: %f. Range min: %f. Min Index: %i", minLaserDist, msg->range_min, laser_min_index);


}

// FIRST EXAMPLE: Odom
void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    // // //fill with your code
    // NOTE: HOw this works is that when "msg" gets updated, this callback function
    // will save it to posX and posY variables. posX and posY are global variables.
    // msg->pose.pose.position.x is the variable that gets updated.

    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    // tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}
// #endif