#include "ros/ros.h"
#include "assignment_2_2024/GetTarget.h" // Include your custom service header
#include "assignment_2_2024/PlanningActionGoal.h" // Include the message header for PlanningActionGoal

// Variables to store the latest x and y values
double target_x = 0.0;
double target_y = 0.0;

// Callback for the topic subscriber
void goalCallback(const assignment_2_2024::PlanningActionGoal::ConstPtr& msg) {
    target_x = msg->goal.target_pose.pose.position.x;
    target_y = msg->goal.target_pose.pose.position.y;

    ROS_INFO("Updated target position: x = %f, y = %f", target_x, target_y);
}

// Service callback function
bool getTargetPosition(assignment_2_2024::GetTarget::Request &req,
                       assignment_2_2024::GetTarget::Response &res) {
    res.x = target_x;
    res.y = target_y;

    ROS_INFO("Service called. Returning position: x = %f, y = %f", res.x, res.y);
    return true;
}

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "get_target_service_node");
    ros::NodeHandle nh;

    // Subscriber to the /reaching_goal/goal topic
    ros::Subscriber goal_sub = nh.subscribe("/reaching_goal/goal", 10, goalCallback);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("get_target", getTargetPosition);

    ROS_INFO("Service node is ready to provide target position.");
    ros::spin();

    return 0;
}

