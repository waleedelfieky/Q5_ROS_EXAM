/*========================================================*/
//includes
#include <ros/ros.h>						// For basic ROS functionality
#include <actionlib/client/simple_action_client.h>		// to be able to interface with the action server
#include <assignment_2_2024/PlanningAction.h>			// auto-generated header file for the custom action messages. 	
#include <nav_msgs/Odometry.h>					// To subscribe to /odom for real-time odometry updates. "it is a standard topic in ROS 1"
#include <assignment_2_2024/RobotState.h>				// Custom message for publishing robot state. "we have created it inside msg directory"
#include <iostream>						// to interact with user throguh UI
/*========================================================*/

//creating alias to the ActionClient using the custom service message and name it Planning Client 
typedef actionlib::SimpleActionClient<assignment_2_2024::PlanningAction> PlanningClient;



//now we have the action client and we do need to do a class that encapsulates the whole client logic
class RobotActionClient
{
public:
	//first define the constructor that would be called when we initilize any object of type RobotActionClient later on
	//we will assign server name "reaching_goal" to the ac object of PlanningClient that we have defined as alias to action client above  
    	RobotActionClient() : ac("reaching_goal", true)
    	{
    		// now wait until the server is connected 
        	ac.waitForServer();
        	//after server is successfully connected now Advertises to the topic named robot_state to publish the robot's state later
        	pub = nh.advertise<assignment_2_2024::RobotState>("robot_state", 10);
        	//we will subscribe to the data comming from /odom too "RobotActionClient::odomCallbac" method would be defined in this class
        	odom_sub = nh.subscribe("/odom", 10, &RobotActionClient::odomCallback, this);
    	}
	/*
    	method that send the goal to the server in our case the goal is x and y
	first we need to know the message type used to send our goal "our goal is defined in the custom action data type
	we will see the type of message in goal section then see it's structure by rosmsg show geometry_msgs/PoseStamped so we can know it's elements*/
	
    	void sendGoal(double x, double y)
    	{
    		//define a planningGoal data type that is exist in the goal section of planning custom service type
        	assignment_2_2024::PlanningGoal goal;
        	//now assign it's elements with our wanted values
        	goal.target_pose.header.frame_id = "map";
        	goal.target_pose.header.stamp = ros::Time::now();
        	goal.target_pose.pose.position.x = x;
        	goal.target_pose.pose.position.y = y;
        	goal.target_pose.pose.orientation.w = 1.0;
    		/*
    		Registers callback functions for done, active, and feedback states.
    		*/
        	ac.sendGoal(goal,
                    boost::bind(&RobotActionClient::doneCb, this, _1, _2),
                    boost::bind(&RobotActionClient::activeCb, this),
                    boost::bind(&RobotActionClient::feedbackCb, this, _1));
    	}
    	//cancel method it just call the cancelGoal inside ac object
    	void cancelGoal()
    	{
        	ac.cancelGoal();
        	ROS_INFO("Goal canceled.");
    	}

private:
	//class members that gonna be used later
    	ros::NodeHandle nh;
    	PlanningClient ac;
    	ros::Publisher pub;
    	ros::Subscriber odom_sub;
    	assignment_2_2024::RobotState robot_state;
    	//define the done call back
    	void doneCb(const actionlib::SimpleClientGoalState &state, const assignment_2_2024::PlanningResultConstPtr &result)
    	{
        	ROS_INFO("Goal finished with state: %s", state.toString().c_str());
    	}
    	//define the actice call back
    	void activeCb()
    	{
        	ROS_INFO("Goal just went active");
    	}
    	//define the feedback call back
    	void feedbackCb(const assignment_2_2024::PlanningFeedbackConstPtr &feedback)
    	{
        	// Placeholder for feedback processing if needed
        	//ROS_INFO("Got Feedback");
        	pub.publish(robot_state);

    	}
    	//define the call back function
    	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    	{
        	robot_state.x = msg->pose.pose.position.x;
        	robot_state.y = msg->pose.pose.position.y;
        	robot_state.vel_x = msg->twist.twist.linear.x;
        	robot_state.vel_z = msg->twist.twist.angular.z;
        	pub.publish(robot_state);
    	}
};

int main(int argc, char **argv)
{
	//
    	ros::init(argc, argv, "robot_action_client");
   	RobotActionClient client;

    	char command;
    	int x, y;
    	//get input from user 
    	while (ros::ok())
    	{
    		std::cout<<"============================================="<<std::endl;
        	std::cout<< "Please enter command"<<std::endl;
        	std::cout<<"s: set"<<std::endl;
        	std::cout<<"c: For Cancel"<<std::endl;
        	std::cout<<"e: For End" << std::endl;
        	std::cin >> command;
        	if (command == 's')
        	{
            		std::cout<<"============================================="<<std::endl;
            		std::cout << "Enter x: ";
            		std::cin >> x;
            		std::cout << "Enter y: ";
            		std::cin >> y;
            		std::cout << "Robot will go to x: " << x << " and y: " << y <<std::endl;
            		std::cout<<"============================================="<<std::endl;
            		client.sendGoal(x, y);
        	}
        	else if (command == 'c')
        	{
            		client.cancelGoal();
        	}
        	else if (command == 'e')
        	{
            		break;
        	}
        	else
        	{
            		std::cout << "You have entered invalid input." << std::endl;
        	}

        	ros::spinOnce(); // Process ROS callbacks without blocking
       	ros::Duration(1).sleep(); // Small sleep to allow console output to refresh properly
    	}

    	return 0;
}

