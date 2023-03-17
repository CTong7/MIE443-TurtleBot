#include <boxes.h> //boxes.cpp is what loads gazebo_coords.xml into our contest file, we won't need to modify this EVER
#include <navigation.h>
#include <robot_pose.h> // runs callback funciton to robot_pose.cpp, by claling a header you call a cpp???
#include <imagePipeline.h>
#include <chrono>
#include <target_pose_calculation.h>
//addd a .h file
#include <target_pose_calculation.cpp>
#include <imagePipeline.cpp>

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0); //RobotPose is a CLASS, we are creating an object called robotPose which takes in 3 inputs, x,y, yaw
    //robot pose is set to 0

    //subscribe to amcl_pose which gives robot pose xyyaw, posecallback is what calls this 
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

        // Part 1: Calculate positions
        //input: boxes.coords file
        //output: NEW x,y,yaw coordinates
        vector<float> pos1=target_pose(Boxes.coords[0]); //pass info of a single box
        vector<float> pos2=target_pose(Boxes.coords[1]);//pass info of a single box
        vector<float> pos3=target_pose(Boxes.coords[2]); //pass info of a single box
        vector<float> pos4=target_pose(Boxes.coords[3]); //pass info of a single box
        vector<float> pos5=target_pose(Boxes.coords[4]); //pass info of a single box

        
        //Part 2: Call travelling salesman
        //input: 5 boxes.coords
        //output: ordered list of positions to travel to, where start of list = 1st position to travel to
        vector <float> ordered_locations={pos1,pos2,pos3,pos4,pos5}; //output 
        std::cout<<ordered_locations<<""; // write out ordered lcoations


        //Part 3: Call move_base
        //input: ordered list of coordinates
        //output 1: move to location and take photos
        //ouput 2: text file of photo labels and lcoations
        for (int i = 0; i<ordered_locations.size(); i++){
            std::cout <<ordered_locations[i]<<" ";
            //moveToGoal takes in Floats 
            xGoal=ordered_locations[i].robot_x;
            yGoal=ordered_locations[i].robot_y;
            phiGoal=ordered_locations[i].robot_yaw;

            bool Navigation::moveToGoal(xGoal,yGoal,phiGoal);

            //Part 4: call image pipeline and take images
            //******INSERT IMAGE PROCESSING CODE HERE****//
        }

      

        
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        int template_id = imagePipeline.getTemplateID(boxes);
        ROS_INFO("The box is: %i", template_id);
        
        ros::Duration(0.01).sleep();
    }
    return 0;
}
