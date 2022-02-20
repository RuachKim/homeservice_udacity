#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects_basic");

  // ROS Node Handler
  ros::NodeHandle n1;
  // Target position publisher
  ros::Publisher target = n1.advertise<geometry_msgs::Pose>("/geometry_target",10);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 5.0;
  goal.target_pose.pose.position.y = 5.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // First Goal
  geometry_msgs::Pose goal1;
  // position of goal1
  goal1.position.x = goal.target_pose.pose.position.x;
  goal1.position.y = goal.target_pose.pose.position.y;
  goal1.orientation.x = 0.0;
  goal1.orientation.y = 0.0;
  goal1.orientation.w = goal.target_pose.pose.orientation.w;
   
  // publish geometry
  target.publish(goal1);
  // ROS INFO; pickup location 
  ROS_INFO("Pickup location for Goal1 x:%.2f, y:%.2f, w:%.2f",
    goal1.position.x,
    goal1.position.y,
    goal1.orientation.w);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot reaches to the goal1");
    // Wait 5 sec after being reached to the point
    ros::Duration(5.0).sleep();
  }
  else{
    ROS_INFO("The robot failed to reach to the goal1 for some reason");
  }

  move_base_msgs::MoveBaseGoal goal_2;

  // set up the frame parameters
  goal_2.target_pose.header.frame_id = "map";
  goal_2.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal_2.target_pose.pose.position.x = 5.0;
  goal_2.target_pose.pose.position.y = -5.0;
  goal_2.target_pose.pose.orientation.w = -1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal2");
  ac.sendGoal(goal_2);

  // First Goal
  geometry_msgs::Pose goal2;
  // position of goal1
  goal2.position.x = goal_2.target_pose.pose.position.x;
  goal2.position.y = goal_2.target_pose.pose.position.y;
  goal2.orientation.x = 0.0;
  goal2.orientation.y = 0.0;
  goal2.orientation.w = goal_2.target_pose.pose.orientation.w;
   
  // publish geometry
  target.publish(goal2);
  // ROS INFO; pickup location 
  ROS_INFO("Pickup location for Goal2 x:%.2f, y:%.2f, w:%.2f",
    goal2.position.x,
    goal2.position.y,
    goal2.orientation.w);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the robot reaches to the goal2");
    ros::Duration(5.0).sleep();
  }
  else{
    ROS_INFO("The robot failed to reach to the goal2 for some reason");
  }
  return 0;
}
