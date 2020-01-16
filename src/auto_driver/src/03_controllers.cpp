/**
 * 03_controllers implements several control laws for a car-like robot
 *
 * Publishes to:
 *    /AutoNOMOS_mini/manual_control/speed    (std_msgs/Int16)
 *    /AutoNOMOS_mini/manual_control/steering (std_msgs/Int16)
 * Subscribes to:
 *    /AutoNOMOS_mini/real_pose_from_gazebo   (geometry_msgs/Pose2D)
 *    /auto_driver/next_pose	(geometry_msgs/Pose2D) next pose in path
 *    /auto_driver/set_control_law (std_msgs/String) either of
 *        "MoveToPoint", "FollowPath", "MoveToPose", "FollowLine"
 * Services provided:
 *    /auto_driver/is_goal_set	(true if robot is trying to get to a goal)
 *    /auto_driver/reset_path   (to reset the path)
 *
 */

#include "03_robot.cpp"

#include <deque>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>


Robot *sim_robot;
deque<geometry_msgs::Pose2D> path;
double goal_x;
double goal_y;
double goal_theta;
bool goal_set = false;

#define ros_buffer_size 1 // number of messages to buffer before starting to discard
#define ros_loop_rate 100 // 2Hz is the frequency of the ros loop

void PoseCallback(const geometry_msgs::Pose2D& msgIn) {
  sim_robot->JumpTo(msgIn.x,msgIn.y,msgIn.theta);
}

void NextPoseCallback(const geometry_msgs::Pose2D& msgIn) {
  path.push_back(msgIn);
}

bool ResetPathCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  path.clear();
  goal_set = false;
  return true;
}

void SetControlLawCallback(const std_msgs::String& msgIn) {
  path.clear();
  goal_set = false;
  string controller= msgIn.data;
  sim_robot->SetControlLaw(controller);
}

bool IsGoalSetCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
  resp.success=goal_set;
  return goal_set;
}


int main(int argc, char**argv) {
  ros::init(argc, argv, "controllers"); // Node 03_controllers
  ros::NodeHandle nh; // Main access point to communications with ROS

  // publishers for speed and steering
  ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", ros_buffer_size);
  ros::Publisher steering_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", ros_buffer_size);

  // service to indicate paths completed
  ros::ServiceServer completed_srv = nh.advertiseService("auto_driver/is_goal_set", IsGoalSetCallback);

  // subscriber to car pose from simulator
  ros::Subscriber pose_sub = nh.subscribe("/AutoNOMOS_mini/real_pose_from_gazebo", ros_buffer_size, PoseCallback);

  // subscriber to next pose in path
  ros::Subscriber  next_pose_sub = nh.subscribe("/auto_driver/next_pose", ros_buffer_size, NextPoseCallback);

  // subscriber to set controller
  ros::Subscriber controller_sub = nh.subscribe("/auto_driver/set_control_law", ros_buffer_size, SetControlLawCallback);
  
  // service to handle reset path requests
  ros::ServiceServer reset_path_srv = nh.advertiseService("auto_driver/reset_path", ResetPathCallback);
  

  sim_robot = new Robot(0,0,0,"Sim",&speed_pub,&steering_pub);

  ros::Rate loop_rate(ros_loop_rate);
  double delta_t = 1.0/ros_loop_rate;

  while (ros::ok()) {
    bool goal_achieved = false;
    if (goal_set) {      
      goal_achieved = sim_robot->ComputeControls();
      sim_robot->PublishControls();
    } else { // stop robot
      sim_robot->MapControls(0.0,0.0);
      sim_robot->PublishControls();
    }

    if ((!goal_set || goal_achieved)) { // Next goal
      if (path.size() !=0) { // get next goal from path
	sim_robot->SetGoal(path[0].x, path[0].y, path[0].theta);
	goal_set = true;
	path.pop_front(); // Remove goal from path
      } else { // last goal has been achieved
	goal_set = false;
      }
    }
    
    ROS_INFO_STREAM(*sim_robot);
        
    // remaining necessary calls for ROS loop and callbacks
    loop_rate.sleep(); // waits what necessary to keep loop_rate
    ros::spinOnce(); // handles callbacks
  }
  return 0;
}
