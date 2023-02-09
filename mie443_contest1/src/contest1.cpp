/* How to Launch *//*

Gazebo:
roslaunch mie443_contest1 turtlebot_world.launch world:=1

RViz:
roslaunch turtlebot_rviz_launchers view_navigation.launch

GMapping:
roslaunch turtlebot_gazebo gmapping_demo.launch

Teleop:
roslaunch turtlebot_teleop keyboard_teleop.launch

Source Code:

cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun mie443_contest1 contest1

How to Connect with Real Robot:

roslaunch turtlebot_bringup minimal.launch
roslaunch turtlebot_navigation gmapping_demo.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
rosrun mie443_contest1 contest1

Steps to push to Github:
1. How to add changes

git add <your files>

2. Commit changes

git commit -m "your message

3. Push to main

git push origin main

*/
#include "callback_functions.cpp" // callback_functions.h doesn't work ...

// #include "move_functions.h"

void e_stop(int time) { // why do we have this?
    ros::NodeHandle nh_custom;
    ros::Publisher vel_pub_custom = nh_custom.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel_forward;

    ros::Subscriber bumper_sub_2 = nh_custom.subscribe("mobile_base/events/bumper", 5, &bumperCallback);
    ros::Subscriber laser_sub_2 = nh_custom.subscribe("scan", 2, &laserCallback);
    ros::Subscriber odom_2 = nh_custom.subscribe("odom", 1, &odomCallback);    
    
    ros::Rate loop_rate_2(100);

    vel_forward.angular.z = 0;
    vel_forward.linear.x = 0;
    vel_forward.linear.y = 0;
    
    // Start Timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Stop the robot until secondsElapsed > time
    while(secondsElapsed < time){
        vel_pub_custom.publish(vel_forward);
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
    }

}
/* RANDOM WALK SKELETON

while:
    0. spinOnce
    1. Check if way is clear
    2. IF CLEAR: Move forward at 0.2 m/s
    3. ELSE: Rotate 30 degrees

*/

void move_x_meters(double meters){
    // Define Publishers and Subscribers
    // Take current position reading from odom
    // Travel backwards until distance travelled is meters long
    // Stop
    // End function

    /* Consider deleting these and only using subscribers/publsihers from int main */
    ros::NodeHandle nh_custom;
    ros::Publisher vel_pub_custom = nh_custom.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel_forward;

    ros::Subscriber bumper_sub_2 = nh_custom.subscribe("mobile_base/events/bumper", 5, &bumperCallback);
    ros::Subscriber laser_sub_2 = nh_custom.subscribe("scan", 2, &laserCallback);
    ros::Subscriber odom_2 = nh_custom.subscribe("odom", 1, &odomCallback);    
    
    ros::Rate loop_rate_2(100);

    // Take current odom reading
    float old_x = posX;
    float old_y = posY;
    float init_pose[] = {old_x,old_y};
    double distance = 0.0;

    // Keep moving 
    if (meters>0){
        while (distance < meters){
            // Must spin to update odom values
            ros::spinOnce(); // More efficient way? maybe just ask odom_2 for info
            // Can we somehow wait here until odom_2 returns a value?

            distance = sqrt(pow(init_pose[0]-posX,2.0) + pow(init_pose[1]-posY,2.0));

            ROS_INFO("My distance: %f", distance);
            vel_forward.linear.x=0.2;
            vel_pub_custom.publish(vel_forward);

            //Restricts loop rate to 100 Hz
            loop_rate_2.sleep();
            // For loop rate 5, distance = 0.345 m, but actual distance travelled = 0.5 m
            // For loop rate 2, distance = 0.285 m
            // For loop rate 100
        }

        // Brake
        vel_forward.linear.x=0.0;
        vel_forward.linear.y=0.0;
        vel_pub_custom.publish(vel_forward);
    }

    if (meters<0){
        while (distance > meters){
            // Must spin to update odom values
            ros::spinOnce(); // More efficient way? maybe just ask odom_2 for info
            // Can we somehow wait here until odom_2 returns a value?

            distance = sqrt(pow(init_pose[0]-posX,2.0) + pow(init_pose[1]-posY,2.0));

            ROS_INFO("My distance: %f", distance);
            vel_forward.linear.x=-0.2;
            vel_pub_custom.publish(vel_forward);

            //Restricts loop rate to 100 Hz
            loop_rate_2.sleep();
            // For loop rate 5, distance = 0.345 m, but actual distance travelled = 0.5 m
            // For loop rate 2, distance = 0.285 m
            // For loop rate 100
        }

        // Brake
        vel_forward.linear.x=0.0;
        vel_forward.linear.y=0.0;
        vel_pub_custom.publish(vel_forward);
    }
        
}

void rotate(double desired_angle)
{

    /* Consider deleting these and only using subscribers/publsihers from int main */
    ros::NodeHandle nh_custom;
    ros::Publisher vel_pub_custom = nh_custom.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel_forward;

    ros::Subscriber bumper_sub_2 = nh_custom.subscribe("mobile_base/events/bumper", 5, &bumperCallback);
    ros::Subscriber laser_sub_2 = nh_custom.subscribe("scan", 2, &laserCallback);
    ros::Subscriber odom_2 = nh_custom.subscribe("odom", 1, &odomCallback);    
    
    ros::Rate loop_rate_2(100);

    // Take current odom reading
    float old_yaw = yaw;
    double angle = 0.0; //in Rad.

    // Keep moving
    // For CCW
    if (desired_angle > 0){
        while (angle < desired_angle){
        // Must spin to update odom values
        ros::spinOnce(); // More efficient way? maybe just ask odom_2 for info
        
        angle = yaw - old_yaw;

        if (angle < 0){
            // Add 360 deg to correct for when robot rotates past 0 degrees.
            angle += 2*M_PI;
        }

        ROS_INFO("My angle turned CCW: %lf", RAD2DEG(angle));
        
        //Turn Counterclockwise
        vel_forward.angular.z=0.4;

        vel_pub_custom.publish(vel_forward);

        //Restricts loop rate to 100 Hz
        loop_rate_2.sleep();
    
    }
    }

    // For CW
    if (desired_angle < 0){

        while (angle > desired_angle){
        // Must spin to update odom values
        ros::spinOnce(); // More efficient way? maybe just ask odom_2 for info
        if ( yaw < old_yaw){
            angle = yaw - old_yaw; // yaw always less than old_yaw when turning CW

        }
        else{
            angle = yaw - old_yaw -2*M_PI;
        }


        // if (angle > 0){
        //     // Add 360 deg to correct for when robot rotates past 0 degrees.
        //     angle -= 2*M_PI;
        // }

        ROS_INFO("My angle turned CW: %lf", RAD2DEG(angle));
        
        //Turn clockwise
        vel_forward.angular.z=-0.3;

        vel_pub_custom.publish(vel_forward);

        //Restricts loop rate to 100 Hz
        loop_rate_2.sleep();
        }
    }
    

    vel_forward.angular.z=0.0;

    vel_pub_custom.publish(vel_forward);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener"); //can change the node name to anything
    ros::NodeHandle nh;

    // 2 subscribers
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);

        // 1 publisher (creating publisher object)
    //ros::Publisher publishername=node_handle.advertise<message type>(topic_name, queuesize)
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    // Increasing it makes loops run more times per second.
    ros::Rate loop_rate(100);

    //create the message object that you will publish
    //it has class gemoetry_msgs and is the message type twist
    //the twist message type contains a linear and angular component
    geometry_msgs::Twist vel; //class::<message type> <name of variable>
    /**in this case vel is the name of the variable. We can assign vlaues to it by doing
    vel.linear.x= <some value>
    vel.linear.y= <some value>
    in our code this is assigned at the bototm in the while loop*/
    

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    // ** IMPORTANT: If something is too close to sensor, it will not be detected!
    float laser_min_dist = 0.5; // At least 0.5

    int loopCount=0;
    int print_counter=0;


    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        // ROS_INFO("Min Laser: %f", minLaserDist);
        // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false;
        //for i in range 0,1,2
        uint32_t which_bumper_pressed =3;

        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
            // left_bumper
            // middle_bumper
            // right_bumper

            // cut itout
            if (any_bumper_pressed==true){
                which_bumper_pressed=b_idx;

                break;
            }
            //0 is left, 1 is middle, 2 is right
            //assumes that only one bumper can be pressed at once
        
        }
        ROS_INFO("bumper %i was pressed ",which_bumper_pressed);    
        
        
        // States of the robot
        // ** IMPORTANT: If something is too close to sensor, it will not be detected!
        // True if path in front of robot has enough space to move into.
        bool is_way_clear = (minLaserDist > laser_min_dist); //robot has deadzone between 0.5 and 4.5meters
        bool near_wall;
        bool near_end_of_wall;
        bool is_stuck;
        

        if (near_wall && (is_way_clear && !any_bumper_pressed)) {
            //Wall follower

            //Detect Flat Wall OR obstacle that has length > 1m.

            //Decide to follow left or right
            
            //Stone-Skipping Function

        }

       
        
        //drive foward 
        if (is_way_clear && !any_bumper_pressed) {
            // Execute movement
            angular = 0.0;
            linear = 0.4;
        }

        //what defines if way is clear?
        //minLaserDist > laser_min_dist
        //Section 2: only activates if a bumper is hit
        else if (any_bumper_pressed) {
            
            //Case 1: left bumper is hit
            if(which_bumper_pressed==0){
                vel.angular.z = 0;
                vel.linear.x = 0;
                vel_pub.publish(vel);
                move_x_meters(-0.1);
                rotate(DEG2RAD(-30)); //rotate clockwise ie negative angle

            }
            //Case 2: middle bumper is hit
            else if(which_bumper_pressed==1){
                vel.angular.z = 0;
                vel.linear.x = 0;
                vel_pub.publish(vel);
                move_x_meters(-0.1);
                rotate(DEG2RAD(100)); // rotate 100 degrees counter clock wise
            }

            //Case 3: right bumper is hit
            else{
                vel.angular.z = 0;
                vel.linear.x = 0;
                vel_pub.publish(vel);
                move_x_meters(-0.1);
                rotate(DEG2RAD(30)); // rotate counter clock wise
            }

           
            vel_pub.publish(vel);
          

        }

        else {
            //probability it turns left 30%
            // probability it turns right 70%
            // rotate left 30, rotate right 90
            // vel.angular.z = 1;
            vel.linear.x = 1;
            // rotate(DEG2RAD(30));
        }

        if (print_counter>50){ //we want to print once every second, the loop runs 100 times per second
            //printLaser()
            ROS_INFO("Position: (%f, %f) Orientation: %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
            print_counter=0;
        }
        print_counter++;

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // next time work on different brnaches
        // we normally merge branches
        // normally work indepdently and merge to main



        // Update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        
        loop_rate.sleep();
        loopCount++;
    }

    return 0;
}


        // // Basic Timer
        // std::clock_t start2;
        // double duration;
        // start2 = std::clock();
        // while ((std::clock() - start2)/CLOCKS_PER_SEC < 1){
            
        //     ROS_INFO("Waited %f Seconds.",(float) (std::clock() - start2)/CLOCKS_PER_SEC);

        // }

        // //
        // // Check if any of the bumpers were pressed.
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
        // any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }
        // //
        
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
