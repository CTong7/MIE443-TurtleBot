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

            distance = -sqrt(pow(init_pose[0]-posX,2.0) + pow(init_pose[1]-posY,2.0));

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
    float angle = 0.0; //in Rad.

    // Keep moving
    // For CCW
    if (desired_angle > 0){
        while (angle < desired_angle){
        // Must spin to update odom values
        ros::spinOnce(); // More efficient way? maybe just ask odom_2 for info
        
        angle = yaw - old_yaw;

        if ( yaw >= old_yaw - DEG2RAD(1)){ // Subtract small angle because yaw sometimes decreases and triggers else condition.
            angle =  yaw -  old_yaw; // yaw always greater than old_yaw when turning CW

        }
        else{
            angle =  yaw -  old_yaw + 2*M_PI;
        }

        ROS_INFO("My angle turned CCW: %lf", RAD2DEG(angle));
        
        //Turn Counterclockwise
        vel_forward.angular.z=0.5;

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

    
        if ( yaw <= old_yaw +DEG2RAD(1)){ // =< so that when yaw == old_yaw, angle doesn't blow up to -360.
            angle = yaw - old_yaw; // yaw always less than old_yaw when turning CW

        }
        else{
            angle =  yaw -  old_yaw -2*M_PI;
        }


        // if (angle > 0){
        //     // Add 360 deg to correct for when robot rotates past 0 degrees.
        //     angle -= 2*M_PI;
        // }

        ROS_INFO("My angle turned CW: %lf", RAD2DEG(angle));
        
        //Turn clockwise
        vel_forward.angular.z=-0.4;

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
    float laser_min_dist = 0.6; // At least 0.5

    int loopCount=0;
    int print_counter=0;


    //////////// 360 Degree Scan Variables ////////////
    // Initialize distance travelled by robot
    double dist = 0.0;
    // Define 360 scan distance increment in m
    int dist_inc = 6;
    // Initialize distance multiplier for 360 degree scan
    int dist_mult = 1;
     // Initialize variables for tracking accumulated distance travelled
    float prev_x = 0;
    float prev_y = 0;
    float accum_distance=0;
    float stuck_distance = 0.0;
    int shimmy_count = 0;
    bool has_detected_wall = false;

    //counting the number of each occurence
    int num_randomlefts=0;
    int num_randomrights=0;
    int num_shimmys=0;
    int num_rotates=0;
    int num_recoveries =0;



    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        // ROS_INFO("Min Laser: %f", minLaserDist);
        // ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist);
        
        /* Problems from sim trial #1:
        Right and left random turn seem to execute even when nothing is in front of the laser
        360 rotations sometimes don't complete. They only turn 90 degrees.
        random 180 degree rotations?
        Print num shimmy recoveries

        Fixes: changed rotate to be better.

        */ 

       /* Sim trial 2
       Rotate() is still BUGGED

       
       */

      /*Sim trial 4
        it would be nice if our robot could turn in the right direction at corners
        We want to seek open space. When it is at a corner it should follow the wall and turn according to it.
      
      */

        // Calculate the accumulated distance travelled

        accum_distance += sqrt(pow(prev_x-posX,2.0) + pow(prev_y-posY,2.0));


        prev_x = posX;
        prev_y = posY;
        
        ROS_INFO("Accumulated Distance Travelled: %f", accum_distance);

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
        //laser_mind_dist is defined as 0.5 meters
        bool near_wall;
        bool near_end_of_wall;
        //Random Walk
        int rotation_angle = 30;
        int rotation_direction = 1; //-1 for clockwise

        //Recovery Actions
        bool is_stuck = false;

        
        //SECTION :1
        if (is_way_clear && !any_bumper_pressed) {

            // If the accumulated distance travelled is greater than or equal to a multiple of a specified value
            if (accum_distance >= (float) dist_inc*dist_mult){
                // If the distance travelled is greater than or equal to a multiple of a specified value, then 
                // stop moving, rotate 360 degrees, and increment the distance multiplier
                linear = 0.0;
                rotate(DEG2RAD(270));// built in publish
                //Sleep so that variables has time to catch up
                ros::Duration(0.5).sleep();
                //second rotate
                rotate(DEG2RAD(90));
                dist_mult++;

                num_rotates++;
            }

            else {
                angular = 0.0;
                if (minLaserDist < 0.65){
                    // If close to an obstacle, drive slower
                    linear = 0.1;
                }
                else{
                    // else, drive faster
                    linear = 0.25;
                }
            }
            
        
            // Clear shimmy count if shimmy has not been executed.
            shimmy_count=0;
            has_detected_wall = false;
            
        }
        //SECTION:2
        else if (any_bumper_pressed){

            //insert alaa

            vel.angular.z = 0;
            vel.linear.x = 0;
            vel_pub.publish(vel);

            //Case 1: left bumper is hit
            if(which_bumper_pressed==0){

                // back off while facing wall
                move_x_meters(-0.2);

                // Turn back to face previous direction
                rotate(DEG2RAD(-40)); //rotate clockwise ie negative angle

            }
            //Case 2: middle bumper is hit
            else if(which_bumper_pressed==1){

                move_x_meters(-0.1);
                rotate(DEG2RAD(90)); // rotate 100 degrees counter clock wise
            }

            //Case 3: right bumper is hit
            else{

                move_x_meters(-0.2);
                rotate(DEG2RAD(40)); // rotate counter clock wise
            }
            
            
        
            // Clear shimmy count if shimmy has not been executed.
             shimmy_count = 0;
             //Clear has wall detected if wall has not been detected twice in a row
             has_detected_wall = false;
            
        }
        //SECTION :3
        else {
            ROS_INFO("Laser Min Index: %i", laser_min_index);
            ROS_INFO("Lower : %i",(int) nLasers/10);
            //Run RANDOM TURN
            //if the shortest distance is in the middle (ie, robot is roughly perpendicular to wall) 

            if((laser_min_index > (int) nLasers/10) && (laser_min_index <  (int)nLasers*9/10)){ //6 degrees shimmy = 6/60 * nLasers
                // greater than 12 degrees less than 48 degrees then the min distance is in the center
            
                // If has_detected_wall has not been cleared, then turn 180 instead of 90
                // The robot is in a corner, and this code 
                // prevents the robot from turning 90 twice in a row (resulting in a 180).

                if (has_detected_wall && minLaserDist < 0.7){
                    rotate(DEG2RAD(180));
                    has_detected_wall = false;
                    num_randomlefts++;
                    // Go back to while loop start
                    continue;
                }
                //probability generator that picks right or left
                int random_bool=rand()%2; //returns 0 or 1 as an integer
                ROS_INFO("boolean is %i",random_bool);
            
                //if random_bool is 0, turn left 90
                if (random_bool==0){
                    rotate(DEG2RAD(90));
                    num_randomlefts++;
    
                }

                //turn right 90
                else {
                    rotate(DEG2RAD(-90));
                    num_randomrights++;
    
                }
                
                // If robot has random turned due to a wall, set this to be true.
                // On the next loop, if this is still true then the robot is in a corner.
                has_detected_wall = true;
        
                //Clear shimmy count if shimmy has not been executed.
                shimmy_count = 0;
            }

            //RUN SHIMMY
            else{

                // Check if stuck. Continue if not.
                if (shimmy_count > 10){
                    
                    // Recovery Code
                    rotate(DEG2RAD(180));

                    num_recoveries ++;

                
                }
                
                    //If way is not clear, stop and rotate 30
                // First check if there is open space on the left and if there is more than on right side
                if (laser_min_index < (int)nLasers/10){ // slightly less than max possible counts
                ROS_INFO("LMI Less than: %i", laser_min_index);

                    
                    rotate(DEG2RAD(7.5)); // Reduced to 7.5 degrees to let the robot
                    // shimmy through tight spaces
                    num_shimmys++;
                    
                }
                else if (laser_min_index > (int)nLasers*9/10){
                    ROS_INFO("LMI Greater than: %i", laser_min_index);
                    
                    rotate(DEG2RAD(-7.5));// Reduced to 7.5 degrees to let the robot
                    // shimmy through tight spaces

                    num_shimmys++;
    

                }


                // // Recovery Code
                shimmy_count +=1;

                has_detected_wall = false;
        

            }

           
        }
           

       

        
        //if (laser_min_index<(int)nLasers/5 )
        // this gets you the left 12 degrees (60/5)
        // index of laser starts on left
        //number of lasers by default is 639, can be changed
        // each one of these lasers has seperate index

        

     

        // if (print_counter>50){ //we want to print once every second, the loop runs 100 times per second
        //     //printLaser()
        //     ROS_INFO("Position: (%f, %f) Orientation: %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
        //     print_counter=0;
        // }
        // print_counter++;


        //Publish velocities
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        ROS_INFO("there have been a total of %i 360 rotations",num_rotates);
        ROS_INFO("there have been a total of %i random left turns",num_randomlefts);
        ROS_INFO("there have been a total of %i random right turns",num_randomrights);
        ROS_INFO("there have been a total of %i shimmys",num_shimmys);
        ROS_INFO("there are %i recoveries", num_recoveries);

    
        // Update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
        loopCount++;
    }

    return 0;
}

