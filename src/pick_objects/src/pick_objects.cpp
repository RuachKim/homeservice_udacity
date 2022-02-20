#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// global var for getting pos
double marker_pose_x = 0;
double marker_pose_y = 0;
// Callback to subscribe marker
void marker_pos(const visualization_msgs::Marker::ConstPtr& msg)
{
  marker_pose_x = msg->pose.position.x; 
  marker_pose_y = msg->pose.position.y; 

  ROS_INFO("Marker position [%f , %f]", marker_pose_x, marker_pose_y);
}

// Move Base Goal function
void move_base_goal(MoveBaseClient &cli, double pose_x, double pose_y)
{

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = marker_pose_x;
  goal.target_pose.pose.position.y = marker_pose_y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Send Target to reach to robot");
  cli.sendGoal(goal);

  // Wait an infinite time for the results
  cli.waitForResult();

  // Check if the robot reached its goal
  if(cli.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot reaches to the goal");
    // Wait 5 sec after being reached to the point
    ros::Duration(5.0).sleep();
  }
  else{
    ROS_INFO("The robot failed to reach to the goal for some reason");
  }

}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // ROS Node Handler
  ros::NodeHandle n;
  // Target position publisher
  //ros::Publisher target = n.advertise<geometry_msgs::Pose>("/geometry_target",10);

  // Marker Subscriber
  ros::Subscriber marker_sub = n.subscribe("visualization_marker", 10, marker_pos); 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::spinOnce();
 
  // sending robot to reach the goal
  move_base_goal(ac, marker_pose_x, marker_pose_y);

  sleep(5);
  
  ros::spinOnce();
 
  // sending robot to reach the goal
  move_base_goal(ac, marker_pose_x, marker_pose_y);


  return 0;
}
