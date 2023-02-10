// #ifndef CALLBACKS // Do I need to do this? I have #ifndef in callback_functions.h already
// #define CALLBACKS // If i have function prototypes in a header file, do I still need to ifndef the implementations?
// Maybe I don't, but I am a little suspicious of my #include callback_functions.cpp line in contest1.cpp
#include "callback_functions.h"
// TODO: Use #ifndef

// SECOND EXAMPLE: Bumper
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//fill with your code
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
    //ROS_INFO("Bumper #%lf pressed.", bumper);
}



void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//fill with your code
    // nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    // desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
    // ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

    minLaserDist = std::numeric_limits<float>::infinity(); //setting minLaserDistance to positive infinity

    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
    // Checks if desiredAngle is less than max range of the laser scanner.
    // If true, cut out unnecessary laser scanner readings
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            if (!std::isinf(msg->ranges[laser_idx])){
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            }
        }
    }
    else {
        int mindex=0;//andrew

        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);

            //reassign mindex if a smaller is found
            if (msg->ranges[laser_idx]<msg->ranges[mindex]){
                mindex=laser_idx;

            }

        }

        laser_min_index=mindex;
        //msg-->ranges is a pointer, you want to access the ranges property of the msg object
        //msg points ot a message object that stores the ranges value
        // this is different than the vel.linear.x, since vel is an object, whereas msg is an object
    }
    //ROS_INFO("the range is %d",msg->ranges);

    //ROS_INFO("Min Laser Dist: %f", minLaserDist);


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

//create a funciton that prints linear and angular velcoity
//create a funciton that prints laser readings
//create a funciotn that prints bumpers