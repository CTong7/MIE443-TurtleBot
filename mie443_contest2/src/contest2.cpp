/* Launch Code:

Gazebo:
roslaunch mie443_contest2 turtlebot_world.launch world:=1

AMCL:
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/andrew/test/src/MIE443-TurtleBot/mie443_contest2/maps/map_1.yaml

RViz:
roslaunch turtlebot_rviz_launchers view_navigation.launch

Contest2:
catkin_make
source devel/setup.bash
rosrun mie443_contest2 contest2

*/
#include <boxes.h> //boxes.cpp is what loads gazebo_coords.xml into our contest file, we won't need to modify this EVER
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0); //RobotPose is a CLASS, we are creating an object called robotPose which takes in 3 inputs, x,y, yaw
    //robot pose is set to 0


    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Initialize box coordinates and templates
    //loads the location of the boxes from an xml file
    //load_coords is a BOOLEAN (TRUE OR FLASE)
    //load_templates is a BOOLEAN

    Boxes boxes; //create a boxes object of class Boxes
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n); // Is there a variable/object somewhere that stores the actual image?

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;
    
    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce(); //callback all subscribers

        
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        
        int template_id = imagePipeline.getTemplateID(boxes);

        ROS_INFO("The box is: %i", template_id);
        
        ros::Duration(0.01).sleep();
    }
    return 0;
}
