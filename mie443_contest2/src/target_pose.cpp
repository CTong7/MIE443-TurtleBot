#include <target_pose.h>

using namespace std;

//custom class called target_robot_pose
//creating a function called target_pose
//should just return a vector list of x,y and yaw position
// INPUT = boxes.coords which is a vector float
//bool Navigation::moveToGoal(float xGoal, float yGoal, float phiGoal){

vector<float> targetclass::target_pose(RobotPose robotPose, vector<float> target_object){
    
    //creating points and checking 3 seperate times essentially
    // Calculate point m_0 for a given object using the object's coordinates
    float m=0.5; // radius
    float PIF = std::number::pi_v<float>;

    //target_object is boxes.coords [0] =x, [1]=y, [2]=angle

    float phi = target_object[2]; // this gets you anlge in radians
    float m_0_x = target_object[0] + m*cos(phi); // x-coordinate of point m_0
    float m_0_y = target_object[1] + m*sin(phi); // y-coordinate of point m_0
    //is boxes .coords in radians

    //define START VARIABLE
	geometry_msgs::PoseStamped start;
    start.header.frame_id="map";
    start.header.stamp = ros::Time::now();
    start.pose.position.x=robotPose.x;
    start.pose.position.y=robotPose.y

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

        
        m_0_x = target_object[0] + m*cos(phi+PIF/6); // x-coordinate of point m_0 add 30 degrees
        m_0_y = target_object[1] + m*sin(phi+PIF/6); // y-coordinate of point m_0

        

        // Check if point is valid
        ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
        nav_msgs::GetPlan srv;
        srv.request.start= start;
        srv.request.goal=goal;
        check_path.call(srv);

        bool pathinvalid= (srv.response.plan.poses.size()<0); //if less than 0 then its not valid

        // check again to see if +45 was valid, if it wasn't
        if (pathinvalid){
            m_0_x = target_object[0] + m*cos(phi-PIF/6); // x-coordinate of point m_0
            m_0_y = target_object[1] + m*sin(phi-PIF/6); // y-coordinate of point m_0

            

            // Check if point is valid
            ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
            nav_msgs::GetPlan srv;
            srv.request.start= start;
            srv.request.goal=goal;
            check_path.call(srv);

            //assumption that by the 3rd call, it must be valid

        }



    }


    int n = 8; // Specify number of possible search points around this specific m_0
    float D = 0.5; // Specify search diameter around point m_0 in meters
    vector <vector <float>> m_i; // nested vector contianing search points around point m_0, creates an 8x2 matrix

    //m_i is the circle of points you are generating, every row is a new point

    // Calculate search points around this specific m_0
    // Filter out inaccessible points (obstructed or off the map)
    //iterate through 2D vector
    // loading the vectors
    

    for(int i = 0; i < n; i++){
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

    for(int i = 0; i < n; i++){
        sum_x = sum_x + m_i[i][0];
        sum_y = sum_y + m_i[i][1];
    }

    // Calculate the centroid of the search points
    //return answer in a vector called centroid
    vector <float> centroid[3];
    centroid[0]= sum_x/n; // load x value into cetnroid
    centroid[1]= sum_y/n; // load y value into centroid
    
    
    // Calculate desired robot orientation
    float phi = target_object[2]; // yaw of the box ie boxes.coords

    //add dot product later

    if(phi >= 0 && phi < PIF){ // if between 0 and 180
    	centroid[2]= phi + PIF; // add 180

    }

    else if(phi >= PIF && phi < 2*PIF){ // if between 180 and 360
        centroid[2] = phi - PIF; // sbuctract 180
    }

    return centroid;

}