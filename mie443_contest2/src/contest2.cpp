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

//This stuff should be a luanch file
*/
#include <boxes.h> //boxes.cpp is what loads gazebo_coords.xml into our contest file, we won't need to modify this EVER
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <cmath>

using namespace std;

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
        // std::cout << "Box coordinates: " << std::endl;
        // std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
        //           << boxes.coords[i][2] << std::endl;
    }
    // Initialize image object and subscriber.
    ImagePipeline imagePipeline(n); // Is there a variable/object somewhere that stores the actual image?

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    // Run until Valid images. Still needs some work.
    // int state = imagePipeline.getTemplateID(boxes);
    // while(state ==-3){
    //     state = imagePipeline.getTemplateID(boxes);
    //     ros::Duration(0.1).sleep();

    // }

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce(); //callback all subscribers
        
        /***YOUR CODE HERE***/
        // 0 = corner rice krispies, 1 = flat toast crunch
        // 2 = empty wall, 3 = ??
        // 4 = ??

        // Hardcode coordinates
        vector<vector<float>> coords {
            {-1.2, 1.5, 2.36},  // rice krispies north
            {0.4, 1, 3.14},  // Toast Krunch
            {-0.107, -0.905, 1.57},  // Empty Box
            {2.4, -0.9, 3.93}, // Rice krispies south 
            {1.9, 1.5, 1.57}, // Raisin bran
            
        };
        // Box 0:
        // Fail -- 


        vector<int> box_id_visited(3);

        for (int i = 0; i < 5; i++){
            cout << "Box Num: " << i <<endl;
            float xGoal = boxes.coords[i][0]; // Distance away/towards the box
            float yGoal = boxes.coords[i][1]; // Distance left/right from the box
            float phiGoal = boxes.coords[i][2];
            cout << "*****************" <<endl;
            cout << "Box Coords: " <<endl;
            cout << "x: " << xGoal << endl;
            cout << "y: " << yGoal << endl;
            cout << "phi: " << phiGoal << endl;
            cout << "*****************" <<endl;
            // Want to move 0.5 m away form face of box
            xGoal += cos(phiGoal)*0.5;
            yGoal += sin(phiGoal)*0.5;

            phiGoal += M_PI;
            
            cout << "*****************" <<endl;
            cout << "x : " << xGoal << endl;
            cout << "y : " << yGoal << endl;
            cout << "phi : " << phiGoal << endl;
            cout << "*****************" <<endl;

            // float xGoal = coords[i][0];
            // float yGoal = coords[i][1];
            // float phiGoal = coords[i][2];

            Navigation::moveToGoal(xGoal, yGoal, phiGoal); 
            ros::spinOnce(); //Get new image
            ros::Duration(0.1).sleep(); //wait
            int template_id = imagePipeline.getTemplateID(boxes); //How to initalize camera?
            // First few images taken are always Invalid. Need to wait for "Initialized OpenCL Runtime"

            ROS_INFO("The box is: %i", template_id);
            box_id_visited.push_back(template_id);
        }
        
        // //Write to text file
        // ofstream myfile;
        // myfile.open ("example.txt");
        // myfile << "Writing this to a file.\n";

        // for (int i=0; i< box_id_visited.size; i++){
        //     myfile << "Box ID: " << box_id_visited[i] <<endl;
        //     myfile << "Box Location: " << box_id_visited[i] <<endl;
        //     myfile << "Box ID: " << box_id_visited[i] <<endl;
            
        // }
        // myfile.close();

        // break;
        ros::Duration(0.01).sleep();
    }
    return 0;
}
