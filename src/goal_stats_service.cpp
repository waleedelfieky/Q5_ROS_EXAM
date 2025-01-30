/*=========================================================*/
#include "ros/ros.h"
#include "assignment_2_2024/GetGoalStats.h" // Include your custom service header
/*=========================================================*/

/*=========================================================*/
// define varibales
int goals_reached = 0;
int goals_cancelled = 0;
/*=========================================================*/
/*=========================================================*/
// Service callback function
bool getGoalStats(assignment_2_2024::GetGoalStats::Request &req,
                  assignment_2_2024::GetGoalStats::Response &res) {
    res.goals_reached = goals_reached;
    res.goals_cancelled = goals_cancelled;

    ROS_INFO("goals reached: %d, goals cancelled: %d", res.goals_reached, res.goals_cancelled);
    return true;
}
/*=========================================================*/

int main(int argc, char **argv) {
/*=========================================================*/
    // Initialize the ROS node
    ros::init(argc, argv, "goal_stats_service_node");
    ros::NodeHandle nh;
/*=========================================================*/
    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("get_goal_stats", getGoalStats);
/*=========================================================*/
    ROS_INFO("Service node is ready to provide goal statistics.");
    ros::spin();
/*=========================================================*/
    return 0;
}
