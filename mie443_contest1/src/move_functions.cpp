// #include "move_functions.h"
// #include "callback_functions.h"

// // // SECOND EXAMPLE: Bumper
// // void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
// // {
// // 	//fill with your code
// //     // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
// //     bumper[msg->bumper] = msg->state;
// // }

// // void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
// // {
// // 	//fill with your code
// //     // nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
// //     // desiredNLasers = DEG2RAD(desiredAngle)/msg->angle_increment;
// //     // ROS_INFO(Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);

// //     minLaserDist = std::numeric_limits<float>::infinity();
// //     nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
// //     desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment);
// //     ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers);
    
// //     if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
// //         for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
// //             minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
// //         }
// //     }
// //     else {
// //         for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
// //             minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
// //         }
// //     }

// // }

// // // FIRST EXAMPLE: Odom
// // void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
// // {
// //     // // //fill with your code
// //     posX = msg->pose.pose.position.x;
// //     posY = msg->pose.pose.position.y;
// //     yaw = tf::getYaw(msg->pose.pose.orientation);
// //     tf::getYaw(msg->pose.pose.orientation);
// //     ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
// // }

// // Actual functions

// void move_x_meters(int x){
//     // Subscribers
//     ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
//     ros::NodeHandle nh_custom;
//     ros::Publisher vel_pub_custom = nh_custom.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
//     geometry_msgs::Twist vel_rotate;
    
// }