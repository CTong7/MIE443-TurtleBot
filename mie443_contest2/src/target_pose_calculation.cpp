#include <nav_msgs/GetPlan.h>
// Define class for storing target pose of robot
class target_robot_pose_class{
  public:
     double robot_x, robot_y, robot_yaw;
};

// Target Pose Calculation Function
float target_object[] = boxes.coords[0]
float phi = target_object[2]


target_robot_pose target_pose(target_object_class target_object){

    // Calculate point m_0 for a given object using the object's coordinates
    m_0_x = target_object.object_x + m*cos(phi); // x-coordinate of point m_0
    m_0_y = target_object.object_y + m*sin(phi); // y-coordinate of point m_0

	// Store current robot x,y position
	start.pose.position.x = robotPose.x;
	start.pose.position.y = robotPose.y;

	// Enter candidate goal position calculated above
	goal.pose.position.x = m_i[i][1]; // input goal x coordinate 
	goal.pose.position.y = m_i[i][2]; // input goal y coordinate

	// Check if point is valid
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    check_path.call(srv);

    if srv.response.plan.poses.size<0;{
         // Calculate point m_0 for a given object using the object's coordinates
        m_0_x = target_object.object_x + m*cos(phi+45); // x-coordinate of point m_0
        m_0_y = target_object.object_y + m*sin(phi+45); // y-coordinate of point m_0
        // Enter candidate goal position calculated above
        goal.pose.position.x = m_i[i][1]; // input goal x coordinate 
        goal.pose.position.y = m_i[i][2]; // input goal y coordinate

        // Check if point is valid
        ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
        nav_msgs::GetPlan srv;
        srv.request.start = start;
        srv.request.goal = goal;
        check_path.call(srv);
    }
    if {srv.response.plan.poses.size<0;{
         // Calculate point m_0 for a given object using the object's coordinates
        m_0_x = target_object.object_x + m*cos(phi-45); // x-coordinate of point m_0
        m_0_y = target_object.object_y + m*sin(phi-45); // y-coordinate of point m_0
        // Enter candidate goal position calculated above
        goal.pose.position.x = m_i[i][1]; // input goal x coordinate 
        goal.pose.position.y = m_i[i][2]; // input goal y coordinate

        // Check if point is valid
        ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
        nav_msgs::GetPlan srv;
        srv.request.start = start;
        srv.request.goal = goal;
        check_path.call(srv);}

    int n = 8; // Specify number of possible search points around this specific m_0
    double D = 0.5; // Specify search diameter around point m_0
    double m_i[n][2]; // Array contianing search points around point m_0

    // Calculate search points around this specific m_0
    // Filter out inaccessible points (obstructed or off the map)
    for(i = 0; i < n; i++){
        m_i[i][1] = m_0_x + D*cos((2*M_PI)/(i+1)); // x-coord of m_i
	m_i[i][2] = m_0_y + D*sin((2*M_PI)/(i+1)); // y-coord of m_i

	// Make request to make_plan service
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    nav_msgs::GetPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    check_path.call(srv);

    	// Return path reachability
	// If "srv.response.plan.poses.size()" is > 0, a path can be obtained, so the statement evaluates to "true"
	// If "srv.response.plan.poses.size()" is < 0, no path can be obtained, so the statement evaluates to "false"
    	return srv.response.plan.poses.size() > 0;

	if(size() == true){
	    // If point is valid, continue iterating through for loop
	    continue;
	}
	else{
	    // If point is not valid, set the m_i point to 0 so it doesn't influence the centroid's location
	    m_i_x[i][1] = 0;
	    m_i_y[i][2] = 0;
	    n--; // Decrement the number of possible search points
	}
    }

    // Calculate the centroid using the non-zero'd points in the m_i array 

    // Sum the x- and y- coords of all the search points
    double sum_x = 0;
    double sum_y = 0;
    for(i = 0; i < n; i++){
	sum_x = sum_x + m_i[i][1];
        sum_y = sum_y + m_i[i][2];
    }

    // Calculate the centroid of the search points
    target_robot_pose.robot_x = sum_x/n;
    target_robot_pose.robot_y = sum_y/n;
    
    // Calculate desired robot orientation
    double phi = boxes.coords[min_index][3]; // Object's orientation
    if(phi >= 0 && phi < M_PI){
    	target_robot_pose.robot_yaw = phi + M_PI;
    }
    else if(phi >= M_PI && phi < 2*M_PI){
        target_robot_pose.robot_yaw = phi - M_PI;
    }

    return target_robot_pose;

}