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

How to save a map from GMapping:
rosrun map_server map_saver -f <your_map_name>

Steps to push to Github:
1. How to add changes

git add <your files>

2. Commit changes

git commit -m "your message"

3. Push to main

git push origin main

Steps to making a new branch and checking it out:
1. git checkout -b new_branch
2. git add .
3. git commit -m "message"
4. git push --set-upstream origin new_branch 

*/



/*////////////// Notes ///////////////
Aaran has mentioned that there would be a lot more obstacles in the final maze than in the current test setup.
I think he just gave us a hint.
*/

/* RANDOM WALK SKELETON

while:
    0. spinOnce
    1. Check if way is clear
    2. IF CLEAR: Move forward at 0.2 m/s
    3. ELSE: Rotate 30 degrees

*/



        /////////////////* Seek Testing: *///////////////////////
        // float angle_increment = (float) 60/639;
        // float angle_to_mindist = (float) laser_min_index * angle_increment;
        // float heading_to_mindist = angle_to_mindist - 30; //Positive if left of robot. Negative if right of robot.

        // ROS_INFO("Heading to min: %f. Min Dist: %f", heading_to_mindist, minLaserDist);
        
        // if (has_acquired_obstacle){
        //     rotate(DEG2RAD(heading_to_mindist));
        //     move_x_meters(minLaserDist-0.6); // move until 0.6 m away from obstacle. // What if overshoots?
        // }


        ///////////////////* RANDOM WALK *////////////////////
        // How to improve:
        // Bias it towards empty spaces. If minlaserdist comes from a specific side, turn to side with more space
        // If minlaserdist is very close to the edge of the field of view, don't need to turn as much.
        
        // How to fix poor mapping? How can we improve our map?
        /*
        
        Poor Mapping is:
        --- Unexplored Areas
        --- Incorrect Wall Placement
        ------ Walls are not drawn at the right angles to each other
        ------ The robot is drawing extra walls where there isn't any
        ------ The robot isn't detecting and drawing some walls
        
        *Would wall-following help with any of these issues?
            *What is wall following?
                *Detects wall
                *Follows along the wall
                *Stops when wall ends or obstacle in the way.
                *Proceed in a new direction to find a new wall to follow.

        */

        // How to make robot traverse tight spaces?? Is it ok to just "peek" through them?

        // How to get robot unstuck if it is trapped in a small region of the map?

        // The RGBD camera appears to be innaccurate at longer distances. Can/Should we filter them out?
        // How to filter out faulty sensor readings from camera?

        /*****
        What should our control method look like?
        Not talking about PID; I mean high-level control

        - Behaviour-Based Control
            *Define behaviours such as wall-following, wall-searching, and recover position.
            *Define states such as Near Wall, Near End of Wall, and Stuck
            *Trigger the behaviours based on states.
            *Decision points will allow us to use planning.

        - Deliberative Control
            *The problem with deliberative control is that we can't sense the goal right away.
            *Need some hybrid approach to gather readings before we deliberately plan how we move.

        - Reactive
            *The problem with reactive control is that it is inflexible and can't adapt easily.
            *Need a base minimum level of planning to allow the robot to escape bad situations.

        - Hybrid (Random Walk with Planning)
            *Hybrid compromises the weaknesses of deliberative and reactive control.
            *After constructing a decent map, switch to planned navigation.

        *****/
        /*****
        Behaviours

        --- Wall-following ---
        *Triggers when Near Wall
        *Also requires minLaserDist > too_close_range and !anybumperpressed.

        Ideas:
        //You want to travel along the wall until the end of the wall is reached.
        //How to detect end of wall?
        //How to travel straight and keep a certain distance from wall?

        Behaviour Start:
        -> When near wall, choose which direction to follow.
        -> While following the wall, maintain constant distance.
        -> Look for these: another wall coming up, edge of wall being followed.
        -> Stop when Near End of Wall is triggered. 
        

        --- Wall-searching ---
        *Triggers when Near End of Wall
        *Also requires minLaserDist > too_close_range and !anybumperpressed.
        
        Ideas:
        //You want to look for a new wall when you reach end of wall. Where are they found?
          Free Spaces
          
        Behaviour Start:
        -> Scan the environment for free spaces
        -> Decide which free space to go to if there is more than one.
        -> Turn robot to face the free space, travel towards free space.
        -> Stop when Near Wall is triggered.

        
        --- Recovery ---
        *Triggers when Stuck
        *Also requires minLaserDist > too_close_range and !anybumperpressed.
      
        Ideas:
        //You want to recover the robot to a location where it can go somewhere else.

        Behaviour Start:
        -> Stop the robot
        -> Think of a new plan to reach outside the immediate area.
        -> Navigate towards outside area

        *****/

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


void move_x_meters(double meters){


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
        
        angle = (double) yaw - (double) old_yaw;

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

bool is_far(float dist){
    if (dist > 1){
        return true;
    }
    else {
        return false;
    }
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

    // Increasing it makes loops run more times per second.
    ros::Rate loop_rate(100);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    // ** IMPORTANT: If something is too close to sensor, it will not be detected!
    float too_close_range = 0.5; // At least 0.5

    //////////// 360 Degree Scan Variables ////////////
    // Initialize distance travelled by robot
    double dist = 0.0;
    // Define 360 scan distance increment in m
    int dist_inc = 1.5;
    // Initialize distance multiplier for 360 degree scan
    int dist_mult = 1;

    int loopCount=0;
    
    // Initialize variables for tracking accumulated distance travelled
    float prev_x = 0;
    float prev_y = 0;
    float accum_distance=0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        ROS_INFO("min Laser: %f", minLaserDist);
        // ROS_INFO("vector min Laser: %f", *std::min_element(laser_scan_array.begin(), laser_scan_array.end()));
        //current pose is posX and posY

        // Calculate the accumulated distance travelled
        prev_x = posX;
        prev_y = posY;
        accum_distance += sqrt(pow(prev_x-posX,2.0) + pow(prev_y-posY,2.0));

        //////////// States of the robot ////////////

            // Check if any of the bumpers were pressed.
            bool any_bumper_pressed = false;
            for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) {
                any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
            }   

            // ** IMPORTANT: If something is too close to sensor, it will not be detected!
            // True if path in front of robot has enough space to move into.
            bool is_way_clear = (minLaserDist > too_close_range);

            bool has_acquired_obstacle = false;
            float acquisition_range = 2.4;
            if (minLaserDist < acquisition_range) {
                has_acquired_obstacle = true;
            }

            bool near_wall = false;
            float follow_range = 0.7;
            if (minLaserDist < follow_range) {
                near_wall = true;
            }
        
            // If robot is too close to an obstacle, path is not clear.
            if (minLaserDist < too_close_range) {
                is_way_clear = false;
            }
        
            /* Ideally, robot follows wall at a distance between too_close_range and follow_range */
            // PID ?

            // if ((minLaserDist < 0.6) && ***SOMETHING*** ) {

            // }

            //// Now robot has sought an obstacle and is 0.7 m away from it, in the center of its vision.
            // Turn left and check for obstacles
        
            // Is mindex inside of the left field of vision?
       
            bool wall_follow_is_blocked = true;
            // if (mindex > (int) 639.0/3.0) {
            //     wall_follow_is_blocked = false;
            // }

            // Check if left field of vision is detecting empty space. If yes, that is wall edge.
            bool wall_follow_is_ending;
            // Check if absence is detected in the middle of the FOV. If distance
            // from indices 220 and 440 are greater than 0.7m (the wall follow distance)
            // then the wall has ended.


            // For Recovery
            bool is_stuck;
            
            //Random Walk
            int rotation_angle = 30;
            int rotation_direction = 1; //-1 for clockwise

            //SECTION :1
            if (is_way_clear && !any_bumper_pressed) {

                // If the accumulated distance travelled is greater than or equal to a multiple of a specified value
                if (accum_distance >= dist_inc*dist_mult){
                    // If the distance travelled is greater than or equal to a multiple of a specified value, then 
                    // stop moving, rotate 360 degrees, and increment the distance multiplier
                    linear = 0.0;
                    rotate(DEG2RAD(180));
                    rotate(DEG2RAD(180));
                    dist_mult++;
                }

                else {
                    angular = 0.0;
                    if (minLaserDist < 1){
                        // If close to an obstacle, drive slower
                        linear = 0.1;
                    }
                    else{
                        // else, drive faster
                        linear = 0.25;
                    }
                }
                
            }

            //SECTION :2
            else {
                //If way is not clear, stop and rotate 30
                // First check if there is open space on the left and if there is more than on right side
                if ((open_space_count_left > nLasers/4) && (open_space_count_left > open_space_count_right)){ // slightly less than max possible counts
                    rotation_direction = 1;
                
                }
                else if ((open_space_count_right > nLasers/4) && (open_space_count_right > open_space_count_left)){
                    rotation_direction = -1;

                }
                else { // if none is true, then pick a random direction.
                    int rand_num = rand()%100;
                    if (rand_num > 50){
                        rotation_direction = -1;

                    }
                    else{
                        rotation_direction = 1;
                    }
                }
                vel.angular.z = 0;
                vel.linear.x = 0;
                vel_pub.publish(vel);

                // How to rotate Clockwise? I still have not tested it.
                rotate(DEG2RAD(rotation_direction*rotation_angle)); // CCW
            }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        
        // Update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        
        loop_rate.sleep();
        loopCount++;
    }

    return 0;
}
