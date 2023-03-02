
#include "callback_functions.h"


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
            if (!std::isinf(msg->ranges[laser_idx]) && !isnan(msg->ranges[laser_idx])){
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            }
        }
    }
    else {
        int mindex=0;//andrew
        float dist_compare = 100.0;


        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            // ROS_INFO("I enterd the For Loop %i times", laser_idx);
            // ROS_INFO("Distance is  %f ", msg->ranges[laser_idx]);

            //first iteration of loop ranges[laser_idx]=NaN
            // therefore if statement will not run
            //2nd iteration ranges[laser_idx]=2meters
            // if statement will run
            // rnages[mindex]=NaN this is true so 2nd if statement runs --> dist_compare=100
            //if 2meters<100meters -->mindex=laser_idx
            if (!std::isinf(msg->ranges[laser_idx]) && !isnan(msg->ranges[laser_idx])){
                minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
                if(isnan(msg->ranges[mindex])){
                    dist_compare = 100.0;
                }

                if (msg->ranges[laser_idx] < dist_compare){
                    mindex=laser_idx;
                    dist_compare = msg->ranges[laser_idx];

                    
                    //ROS_INFO("Minimum Index: %i", mindex);
                }
            }

            //reassign mindex if a smaller is found
            

        }

        laser_min_index=mindex;
        laser_center_view_dist = msg->ranges[nLasers/2];
        // ROS_INFO("Test Laser Index: %f", msg->ranges[20]);
        // ROS_INFO("nLasers: %i", nLasers);
        //msg-->ranges is a pointer, you want to access the ranges property of the msg object
        //msg points ot a message object that stores the ranges value
        // this is different than the vel.linear.x, since vel is an object, whereas msg is an object
    }
    //ROS_INFO("the range is %d",msg->ranges);

    //ROS_INFO("Min Laser Dist: %f", minLaserDist);


}

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
