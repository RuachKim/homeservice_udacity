#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath> // for square
#include <nav_msgs/Odometry.h> // To subscribe current position of robot

using namespace std;
// Structure of Point
struct Point{
  double x,y;
};

// Get current position using callback
double cur_x = 0.0; 
double cur_y = 0.0; 
void getCurPose(const nav_msgs::Odometry::ConstPtr& msg)
{
  cur_x = msg->pose.pose.position.x;
  cur_y = msg->pose.pose.position.x;
  ROS_INFO("Current Pos= [X: [%f], Y: [%f]", cur_x, cur_y);

  return;
}

// Calculate distance from current_pose to edge 
double dist_calc(Point p1)
{
  double diff_x = p1.x - cur_x; 
  double diff_y = p1.y - cur_y;
 
  double res = pow(diff_x,2) + pow(diff_y,2);
   
  return sqrt(res);
} 

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  
  // Subscriber for getting current position
  ros::Subscriber pose_subs = n.subscribe("/odom", 10, getCurPose);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set Start and End points
  struct Point pickup;
  struct Point dropoff;
  
  pickup.x = 0.0;
  pickup.y = 5.0;
  dropoff.x = -2.0;
  dropoff.y = 8.0;

  // Current status
  int status = 0; // 0: pickup, 1: loading, 2: drop-off 

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ROS_INFO("Marker Node");    
    // when current position is close to pickup
    if((dist_calc(pickup) < 0.7) && (status==0)){
	marker.action = visualization_msgs::Marker::ADD;
   	marker.pose.position.x = pickup.x;
   	marker.pose.position.y = pickup.y;
   	marker_pub.publish(marker);

	// status to load(item)
  	status = 1;
        
    }

    // when current position is close to drop-off
    else if((dist_calc(pickup) < 0.3) && (status==1)){
	marker.action = visualization_msgs::Marker::DELETE;
   	marker.pose.position.x = dropoff.x;
   	marker.pose.position.y = dropoff.y;
   	marker_pub.publish(marker);
	
	// status to drop(item)
  	status = 2;
        
    }
    
    // Finished to ship
    else if(status==2){
	marker.action = visualization_msgs::Marker::ADD;
   	marker.pose.position.x = dropoff.x;
   	marker.pose.position.y = dropoff.y;
   	marker_pub.publish(marker);
    } 


//    r.sleep();
  }

  return 0;
}
