/**
 * 03_test.cpp node to test 03_controllers
 *
 * This program selects a control law and a sequence of paths to follow
 * waiting for the robot to finish each
 *
 * Publishes to:
 *    /auto_driver/next_pose	(geometry_msgs/Pose2D) next pose in path
 *    /auto_driver/set_control_law (std_msgs/String) either of
 *        "MoveToPoint", "FollowPath", "MoveToPose", "FollowLine"
 * Subscribes to:
 *    /AutoNOMOS_mini/real_pose_from_gazebo   (geometry_msgs/Pose2D)
 * Services requested:
 *    /auto_driver/reset_path   (to reset the path)
 *    /gazebo/reset_simulation   (to reset simulation and move robot to origin)
 *    /auto_driver/is_goal_set	(to see if robot got to goal)
 */

#include <deque>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

using namespace std;

deque<geometry_msgs::Pose2D> path;

double x, y, theta;

#define ros_buffer_size 1 // number of messages to buffer before starting to discard
#define ros_loop_rate 1 // 2Hz is the frequency of the ros loop

void PoseCallback(const geometry_msgs::Pose2D& msgIn) {
  x = msgIn.x;
  y = msgIn.y;
  theta = msgIn.theta;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "test_controllers"); // Node 03_controllers
  ros::NodeHandle nh; // Main access point to communications with ROS

  // publisher for setting next pose in paths
  ros::Publisher  next_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/auto_driver/next_pose", ros_buffer_size);

  // publisher for setting control law
  ros::Publisher control_law_pub = nh.advertise<std_msgs::String>("/auto_driver/set_control_law", ros_buffer_size);
  
  // service client to reset path requests
  ros::ServiceClient reset_path_clnt = nh.serviceClient<std_srvs::Empty>("auto_driver/reset_path");

  // add a service client to gazebo/reset_simulation
  ros::ServiceClient reset_simulation_clnt = nh.serviceClient<std_srvs::Empty>("gazebo/reset_simulation");

  // add a service client to gazebo/reset_simulation
  ros::ServiceClient is_goal_set_clnt = nh.serviceClient<std_srvs::Trigger>("auto_driver/is_goal_set");

  geometry_msgs::Pose2D new_pose;
  std_msgs::String new_control_law;

  ros::Rate loop_rate(ros_loop_rate);
  double delta_t = 1.0/ros_loop_rate;
  int n_times;
 
  std::vector<string> control_laws = { "FollowPath", "MoveToPoint", "MoveToPose"};
  std::array<array<double, 3>, 8> poses = {{ { 2.0,  2.0,   30*M_PI/180},
					     { 6.0,  3.0,    0*M_PI/180},
					     { 9.0,  0.0,  -45*M_PI/180},
					     { 9.0, -3.0,  -90*M_PI/180},
					     { 7.0, -6.0, -135*M_PI/180},
					     { 3.0, -5.0, -180*M_PI/180},
					     { 2.0, -3.0,  135*M_PI/180},
					     { 0.0,  0.0,   90*M_PI/180},
                                          }};

  for (const auto& control_law: control_laws) {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    reset_path_clnt.call(req, resp);
    reset_simulation_clnt.call(req, resp);
    new_control_law.data = control_law;

    while(control_law_pub.getNumSubscribers()<1) {
      loop_rate.sleep(); // waits what necessary to keep loop_rate
      ros::spinOnce(); // handles callbacks      
    }
    ROS_INFO_STREAM("Requesting new control law: "<< new_control_law.data);
    control_law_pub.publish(new_control_law);
    
    for (const auto& pose: poses) { //iterator pose = poses.begin(); pose != poses.end() && ros::ok(); pose++) {
      new_pose.x = pose[0];
      new_pose.y = pose[1];
      new_pose.theta = pose[2];
      ROS_INFO_STREAM("Requesting new pose: "<< new_pose.x << "," << new_pose.y << "," << new_pose.theta);
      next_pose_pub.publish(new_pose);
      // remaining necessary calls for ROS loop and callbacks
      loop_rate.sleep(); // waits what necessary to keep loop_rate
      ros::spinOnce(); // handles callbacks
      n_times--;
      if (!ros::ok())
	break;
    }

    std_srvs::Trigger::Request req_goal_set;
    std_srvs::Trigger::Response resp_goal_set;
    
    while(is_goal_set_clnt.call(req_goal_set, resp_goal_set)
	  && resp_goal_set.success
	  && ros::ok()) {// wait until path is completed
      loop_rate.sleep();
      ros::spinOnce();
    }
    if (!ros::ok())
      break;
  }

  return 0;
}
