#include <boxes.h> //boxes.cpp is what loads gazebo_coords.xml into our contest file, we won't need to modify this EVER
#include <navigation.h>
#include <robot_pose.h> // runs callback funciton to robot_pose.cpp, by claling a header you call a cpp???
#include <imagePipeline.h>
#include <chrono>
//#include <target_pose.h>
//addd a .h file
//#include "target_pose.cpp"
#include "imagePipeline.cpp"
#include <nav_msgs/GetPlan.h>
#include <vector>
#include <robot_pose.h>
#include <cmath>
#include <math.h>
#include <geometry_msgs/PoseStamped.h> // getplan takes gemoetry messages as an input



float PIF = 3.14159265358979323846;

//custom class called target_robot_pose
//creating a function called target_pose
//should just return a vector list of x,y and yaw position
//INPUT1 = robotPose which is the current pose of the robot
// INPUT2 = boxes.coords which is a vector float
//bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){

std::vector<float> target_pose(RobotPose robotPose, std::vector<float> target_object){
    
    //creating points and checking 3 seperate times essentially
    float m=0.5; // radius

    //Target_object contains the x,y and phi coordinates of all 5 points, its a [5][3] 2D array
    //target_pose should be written as a funciton because you need to call it on 5 seperate points
    
    

    //target_object is boxes.coords [0] =x, [1]=y, [2]=angle
    // Calculate point m_0 for a given object using the object's coordinates
    float m_0_phi = target_object[2]; // this gets you anlge in radians
    float m_0_x = target_object[0] + m*cos(m_0_phi); // x-coordinate of point m_0
    float m_0_y = target_object[1] + m*sin(m_0_phi); // y-coordinate of point m_0
    //is boxes .coords in radians

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
    goal.pose.position.x=m_0_x;
    goal.pose.position.y=m_0_y;

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

    //if the first point we generate is not valid, then we want to add 45 degrees
    if (srv.response.plan.poses.size()<0){
        bool validloop = true;
         // Calculate point m_0 for a given object using the object's coordinates +45 degrees

        
        m_0_x = target_object[0] + m*cos(m_0_phi+PIF/6); // x-coordinate of point m_0 add 30 degrees
        m_0_y = target_object[1] + m*sin(m_0_phi+PIF/6); // y-coordinate of point m_0

        

        // Check if point is valid
        ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
        nav_msgs::GetPlan srv;
        srv.request.start= start;
        srv.request.goal=goal;
        check_path.call(srv);

        bool pathinvalid= (srv.response.plan.poses.size()<0); //if less than 0 then its not valid

        // check again to see if +45 was valid, if it wasn't
        if (pathinvalid){
            m_0_x = target_object[0] + m*cos(m_0_phi-PIF/6); // x-coordinate of point m_0
            m_0_y = target_object[1] + m*sin(m_0_phi-PIF/6); // y-coordinate of point m_0

            

            // Check if point is valid
            ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
            nav_msgs::GetPlan srv;
            srv.request.start= start;
            srv.request.goal=goal;
            check_path.call(srv);

            //assumption that by the 3rd call, it must be valid

        }



    }


    int num_points = 8; // Specify number of possible search points around this specific m_0
    float D = 0.5; // Specify search diameter around point m_0 in meters
    std::vector< std::vector <float>> m_i; // nested vector contianing search points around point m_0, creates an 8x2 matrix

    //m_i is the circle of points you are generating, every row is a new point

    // Calculate search points around this specific m_0
    // Filter out inaccessible points (obstructed or off the map)
    //iterate through 2D vector
    // loading the vectors
    

    for(int i = 0; i < num_points; i++){
        //float circlex= m_0_x + D*cos((2*PIF)/(i+1)); // x-coord of m_i
        //float circley= m_0_y + D*sin((2*PIF)/(i+1)); // y-coord of m_i
        m_i[i][0]= m_0_x + D*cos((2*PIF)/(i+1)); // x-coord of m_i
	    m_i[i][1]= m_0_y + D*sin((2*PIF)/(i+1)); // y-coord of m_i

	    // check if the 8 points are valid
        ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
        nav_msgs::GetPlan srv;
        

        srv.request.start= start;
        srv.request.goal.pose.position.x= m_i[i][0];
        srv.request.goal.pose.position.y= m_i[i][1];
        check_path.call(srv);

        // Return path reachability
        // If "srv.response.plan.poses.size()" is > 0, a path can be obtained, so the statement evaluates to "true"
        // If "srv.response.plan.poses.size()" is < 0, no path can be obtained, so the statement evaluates to "false"
        // if poitns are valid set to 0,0 and then and decrment 
        // if point is valid ie greater 0

        // if the new is valid --> go to next iteration of FOR Loop
        if(srv.response.plan.poses.size() > 0){
            // If point is valid, continue iterating through for loop
            continue;
        }

        // set the new point to 0,0 and decrmeent n so that centroid calculation is not affected by it
        else{
            // If point is not valid, set the m_i point to 0 so it doesn't influence the centroid's location
            
            m_i[i][0] = 0;
            m_i[i][1] = 0;
            n--; // Decrement the number of possible search points
        }
    }

    // Calculate the centroid using the non-zero'd points in the m_i array 

    // Sum the x- and y- coords of all the search points
    float sum_x = 0;
    float sum_y = 0;

    for(int i = 0; i < num_points; i++){
        sum_x = sum_x + m_i[i][0];
        sum_y = sum_y + m_i[i][1];
    }

    // Calculate the centroid of the search points
    //return answer in a vector called centroid
   
    std::vector<float> centroid(3);
    centroid[0]= sum_x/num_points; // load x value into cetnroid
    centroid[1]= sum_y/num_points; // load y value into centroid
    
    
    // Calculate desired robot orientation
   

    //add dot product later

    if(m_0_phi >= 0 && m_0_phi < PIF){ // if between 0 and 180
    	centroid[2]= m_0_phi + PIF; // add 180

    }

    else if(m_0_phi >= PIF && m_0_phi < 2*PIF){ // if between 180 and 360
        centroid[2] = m_0_phi - PIF; // sbuctract 180
    }

    return centroid;

}

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
        //in order to index into boxes.coords[1][2]
        std::vector<float> pos1[3];
        std::vector<float> pos2[3];
        std::vector<float> pos3[3];
        std::vector<float> pos4[3];
        std::vector<float> pos5[3];

        //store robot
        //need to figure robotPose into target_pose

        ///amcl_pose --> is a topic that publishes the current position of robot
        // it will store info in variables phi,x and y
        //std::vector<float> target_pose(RobotPose robotPose, std::vector<float> target_object){
        // returns std::vector<float> centroid(3);

        // store calculated x,y and phi for box 1

        pos1[0]= target_pose(); //loading x value
        pos1[1]= target_pose(); //loading y value
        pos1[2]= target_pose(robotPose,boxes.coords); //loading z value

         // store calculated x,y and phi for box 2
        pos2[0]= target_pose(boxes.coords[1][0]);
        pos2[1]= target_pose(boxes.coords[1][1]);
        pos2[2]= target_pose(boxes.coords[1][2]);

        pos3[0]= target_pose(boxes.coords[2][0]);
        pos3[1]= target_pose(boxes.coords[2][1]);
        pos3[2]= target_pose(boxes.coords[2][2]);

        pos4[0]= target_pose(boxes.coords[3][0]);
        pos4[1]= target_pose(boxes.coords[3][1]);
        pos4[2]= target_pose(boxes.coords[3][2]);

        pos5[0]= target_pose(boxes.coords[4][0]);
        pos5[1]= target_pose(boxes.coords[4][1]);
        pos5[2]= target_pose(boxes.coords[4][2]);

        //Part 2: Call travelling salesman
        //input: 5 boxes.coords
        //output: ordered list of positions to travel to, where start of list = 1st position to travel to
        std::vector<float> ordered_locations={pos1,pos2,pos3,pos4,pos5}; //output 
        //std::cout<<ordered_locations<<""; // write out ordered lcoations




        //Part 3: Call move_base
        //input: ordered list of coordinates
        //output 1: move to location and take photos
        //ouput 2: text file of photo labels and lcoations
        for (float i = 0; i<ordered_locations.size(); i++){
            std::cout <<ordered_locations[i]<<" ";
            //moveToGoal takes in Floats 
            //ordered_locations is a vector
            float xGoal=ordered_locations[i][0];
            float yGoal=ordered_locations[i][1];
            float phiGoal=ordered_locations[i][2];

            bool valid=Navigation::moveToGoal(xGoal,yGoal,phiGoal);

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
