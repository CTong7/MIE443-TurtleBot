/* Launch Code:

Gazebo:
roslaunch mie443_contest2 turtlebot_world.launch world:=1

AMCL (Andrew's laptop):
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/andrew/test/src/MIE443-TurtleBot/mie443_contest2/maps/map_1.yaml

AMCL (Turtlebot Laptop):
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/thursday2023/catkin_ws/src/MIE443-TurtleBot/mie443_contest2/maps/map_1.yaml

AMCL (Turtlebot Laptop IRL):
roslaunch turtlebot_navigation amcl_demo.launch map_file:=/home/thursday2023/catkin_ws/src/MIE443-TurtleBot/mie443_contest2/maps/contest2_thurs.yaml

RViz:
roslaunch turtlebot_rviz_launchers view_navigation.launch

Bringup:
roslaunch turtlebot_bringup minimal.launch

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
#include <fstream>

//Srv
#include <geometry_msgs/PoseStamped.h> // getplan takes gemoetry messages as an input
#include <nav_msgs/GetPlan.h>

using namespace std;

vector<vector<int>> nearest_neighbours(float start_pose_x, float start_pose_y, float start_pose_phi, vector<vector<float>> box_coords){
    /*
    At the beginning of the contest, rearrange the coordinates of the boxes to shorten total distance travelled.
    */
    vector<vector<int>> output ={
        {},
        {},
        {},
        {},
        {}
    };

    // while (output[4] == {}){

    // }

}

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
    // Save initial Pose
    vector<float> starting_pose;

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    starting_pose.push_back(robotPose.x);
    starting_pose.push_back(robotPose.y);
    starting_pose.push_back(robotPose.phi);

    cout << "*********** Starting Pose: **************" <<endl;
    cout << "x: " <<starting_pose[0] << endl;
    cout << "y: " <<starting_pose[1] << endl;
    cout  << "phi: "<<starting_pose[2] <<endl;

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce(); //callback all subscribers
        
        /***YOUR CODE HERE***/
        // 0 = North rice krispies, 1 = Toast crunch
        // 2 = empty wall, 3 = South Rice Krispies
        // 4 = Raisin Bran

        // Hardcode coordinates
        // vector<vector<float>> coords {
        //     {-1.2, 1.5, 2.36},  // rice krispies north
        //     {0.4, 1, 3.14},  // Toast Krunch
        //     {-0.107, -0.905, 1.57},  // Empty Box
        //     {2.4, -0.9, 3.93}, // Rice krispies south 
        //     {1.9, 1.5, 1.57}, // Raisin bran
            
        // };

        vector<int> box_id_visited;
        // Storing coordinates
        vector<vector<float>> coords_visited {
            {},  // rice krispies north
            {},  // Toast Krunch
            {},  // Empty Box
            {}, // Rice krispies south 
            {}, // Raisin bran
            
        };
        const float deg2Rad = M_PI/180;

        for (int i = 0; i < 5; i++){

            cout << "Box Num: " << i <<endl;
            bool loop_break = false;
            bool loop_break2 = false;
            float xGoal;
            float yGoal;
            float phiGoal;


            xGoal = boxes.coords[i][0]; // Distance away/towards the box
            yGoal = boxes.coords[i][1]; // Distance left/right from the box
            phiGoal = boxes.coords[i][2];

            phiGoal += 0;
            xGoal += cos(phiGoal)*0.5;
            yGoal += sin(phiGoal)*0.5;
            phiGoal += M_PI;

                    //define START VARIABLE
                geometry_msgs::PoseStamped start;
                start.header.frame_id="map";
                start.header.stamp = ros::Time::now();
                start.pose.position.x=robotPose.x;
                start.pose.position.y=robotPose.y;

                //define GOAL VARIABLE
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id="map";
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x=xGoal;
                goal.pose.position.y=yGoal;

                // Check if point is valid
                ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
                nav_msgs::GetPlan srv;
                srv.request.start= start;
                srv.request.goal=goal;
                // srv.request.start.pose.position.x=robotPose.x; // Get current position of robot
                // srv.request.start.pose.position.y=robotPose.y;// Get current position of robot
                // srv.request.goal.pose.position.x= m_0_x;
                // srv.request.goal.pose.position.y= m_0_y;
                check_path.call(srv);
                
                if (srv.response.plan.poses.size()>0) {
                    cout << "Path plan successful. Valid Plan exists."<<endl;
                    loop_break2 = true;

                }
            
            for (float rad_dist = 0.5; rad_dist < 1; rad_dist += 0.1){
                
                if (loop_break2) {
                    break;
                }
                
                for (float degree = -30.0; degree <= 30.0; degree += 5.0){

                    cout<< "Radial: " << rad_dist<<endl;
                    cout<< "Deg: " << degree <<endl;
                    xGoal = boxes.coords[i][0]; // Distance away/towards the box
                    yGoal = boxes.coords[i][1]; // Distance left/right from the box
                    phiGoal = boxes.coords[i][2];

                    phiGoal += degree*deg2Rad;
                    xGoal += cos(phiGoal)*rad_dist;
                    yGoal += sin(phiGoal)*rad_dist;
                    phiGoal += M_PI;

                    //define START VARIABLE
                geometry_msgs::PoseStamped start;
                start.header.frame_id="map";
                start.header.stamp = ros::Time::now();
                start.pose.position.x=robotPose.x;
                start.pose.position.y=robotPose.y;

                //define GOAL VARIABLE
                geometry_msgs::PoseStamped goal;
                goal.header.frame_id="map";
                goal.header.stamp = ros::Time::now();
                goal.pose.position.x=xGoal;
                goal.pose.position.y=yGoal;

                // Check if point is valid
                ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
                nav_msgs::GetPlan srv;
                srv.request.start= start;
                srv.request.goal=goal;
                // srv.request.start.pose.position.x=robotPose.x; // Get current position of robot
                // srv.request.start.pose.position.y=robotPose.y;// Get current position of robot
                // srv.request.goal.pose.position.x= m_0_x;
                // srv.request.goal.pose.position.y= m_0_y;
                check_path.call(srv);
                
                if (srv.response.plan.poses.size()>0) {
                    cout << "Path plan successful. Valid Plan exists."<<endl;
                    loop_break = true;
                    cout << "(" << rad_dist << ", "<<degree<<")"<<endl;

                    break;
                }
                

                }

                if(loop_break){
                    break;
                }
            }
            
            // cout << "*****************" <<endl;
            // cout << "Box Coords: " <<endl;
            // cout << "x: " << xGoal << endl;
            // cout << "y: " << yGoal << endl;
            // cout << "phi: " << phiGoal <<endl;
            // cout << "*****************" <<endl;
            // // Want to move 0.5 m away form face of box
            // xGoal += cos(phiGoal)*0.5;
            // yGoal += sin(phiGoal)*0.5;

            // phiGoal += M_PI;
            
            // cout << "*****************" <<endl;
            // cout << "Target Pose: " <<endl;
            // cout << "x : " << xGoal << endl;
            // cout << "y : " << yGoal << endl;
            // cout << "phi : " << phiGoal << endl;
            // cout << "*****************" <<endl;

            // // float xGoal = coords[i][0];
            // // float yGoal = coords[i][1];
            // // float phiGoal = coords[i][2];

            // // Check if path is valid

            // //define START VARIABLE
            // geometry_msgs::PoseStamped start;
            // start.header.frame_id="map";
            // start.header.stamp = ros::Time::now();
            // start.pose.position.x=robotPose.x;
            // start.pose.position.y=robotPose.y;

            // //define GOAL VARIABLE
            // geometry_msgs::PoseStamped goal;
            // goal.header.frame_id="map";
            // goal.header.stamp = ros::Time::now();
            // goal.pose.position.x=xGoal;
            // goal.pose.position.y=yGoal;

            // // Check if point is valid
            // ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
            // nav_msgs::GetPlan srv;
            // srv.request.start= start;
            // srv.request.goal=goal;
            // // srv.request.start.pose.position.x=robotPose.x; // Get current position of robot
            // // srv.request.start.pose.position.y=robotPose.y;// Get current position of robot
            // // srv.request.goal.pose.position.x= m_0_x;
            // // srv.request.goal.pose.position.y= m_0_y;
            // check_path.call(srv);
            // if (srv.response.plan.poses.size()>0) {
            //     cout << "Path plan successful. Valid Plan exists."<<endl;
            // }
            
            // else {
            //     cout << "Path plan failure. Invalid Plan. Try +30" <<endl;
            //     // Check +30
            //     xGoal = boxes.coords[i][0]; // Distance away/towards the box
            //     yGoal = boxes.coords[i][1]; // Distance left/right from the box
            //     phiGoal = boxes.coords[i][2];
                
            //     phiGoal += M_PI/6;
            //     xGoal += cos(phiGoal)*0.50;
            //     yGoal += sin(phiGoal)*0.50;
            //     phiGoal += M_PI;


            //     //define START VARIABLE
            //     geometry_msgs::PoseStamped start;
            //     start.header.frame_id="map";
            //     start.header.stamp = ros::Time::now();
            //     start.pose.position.x=robotPose.x;
            //     start.pose.position.y=robotPose.y;

            //     //define GOAL VARIABLE
            //     goal.header.frame_id="map";
            //     goal.header.stamp = ros::Time::now();
            //     goal.pose.position.x=xGoal;
            //     goal.pose.position.y=yGoal;

            //     // Check if point is valid
            //     ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
            //     nav_msgs::GetPlan srv;
            //     srv.request.start= start;
            //     srv.request.goal=goal;
            //     // srv.request.start.pose.position.x=robotPose.x; // Get current position of robot
            //     // srv.request.start.pose.position.y=robotPose.y;// Get current position of robot
            //     // srv.request.goal.pose.position.x= m_0_x;
            //     // srv.request.goal.pose.position.y= m_0_y;
            //     check_path.call(srv);

            //     if (srv.response.plan.poses.size()>0) {
            //         cout << "_+30 Path plan successful. Valid +30 Plan exists."<<endl;
            //     }

            //     else {
            //         // Go with -30. Assume it works
            //         cout << "+30 Path plan failure. Going with -30 Plan."<<endl;

            //         xGoal = boxes.coords[i][0]; // Distance away/towards the box
            //         yGoal = boxes.coords[i][1]; // Distance left/right from the box
            //         phiGoal = boxes.coords[i][2];
                
            //         phiGoal -= M_PI/6;
            //         xGoal += cos(phiGoal)*0.75;
            //         yGoal += sin(phiGoal)*0.75;
            //         phiGoal += M_PI;
            //     }
            

            // }
            

            // Navigate to chosen point
            Navigation::moveToGoal(xGoal, yGoal, phiGoal); 
        

            // Push coords of current box being visited
            coords_visited[i].push_back(boxes.coords[i][0]);
            coords_visited[i].push_back(boxes.coords[i][1]);
            coords_visited[i].push_back(boxes.coords[i][2]);

            ros::spinOnce(); //Get new image
            ros::Duration(0.1).sleep(); //wait
            int template_id = imagePipeline.getTemplateID(boxes); //How to initalize camera?

            ROS_INFO("The box is: %i", template_id);
            for (int j = 0; j< box_id_visited.size(); j++){
                if (box_id_visited[j] == template_id){
                    template_id+=20;
                }
            }
            box_id_visited.push_back(template_id);
        }
        
        bool has_returned_home = Navigation::moveToGoal(starting_pose[0], starting_pose[1], starting_pose[2]); // Return to starting position
        if (has_returned_home){
            ROS_INFO("Returned to Starting Position.");
        }
        //Write to text file
        ofstream myfile;
        myfile.open ("box_results.txt",ios::in | ios::out| ios::trunc); //trunc deletes contents of old file before writing.
        myfile << "CONTEST 2 TEST RESULTS" <<endl;
        myfile << "======================" <<endl;
        
        for (int i=0; i< box_id_visited.size(); i++){
            myfile << "Box #" << i+1 << ":" <<endl;

            if (box_id_visited[i] ==-2){
                myfile << "EMPTY TAG" <<endl;
            }

            if (box_id_visited[i] > 18){
                box_id_visited[i] -= 20;
                myfile << "DUPLICATE TAG OF ID "<<box_id_visited[i] <<endl;

            }

            myfile << "Tag ID: " << box_id_visited[i] <<endl;
            myfile << "Tag Location: " << "(" <<coords_visited[i][0] << ", "<< coords_visited[i][1] << ", "<< coords_visited[i][2] << ")" <<endl;
            myfile << endl;            
        }
        myfile.close();

        break;
        ros::Duration(0.01).sleep();
    }
    return 0;
}
