/**
 * open_loop_driver.cpp implements an open-loop controller for a car-like robot
 * Subscribes to:
 *
 * Publishes to:
 *    /AutoNOMOS_mini/manual_control/steering (std_msgs/Int16)
 *    /AutoNOMOS_mini/manual_control/speed    (std_msgs/Int16)
 * Subscribes to:
 *    /AutoNOMOS_mini/real_pose_from_gazebo   (geometry_msgs/Pose2D)
 */

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>

geometry_msgs::Pose2D current_pose;

#define ros_buffer_size 10 // number of messages to buffer before starting to discard
#define ros_loop_rate 2 // 2Hz is the frequency of the ros loop

void poseCallback(const geometry_msgs::Pose2D& msgIn) {
  current_pose = msgIn;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "open_loop_driver"); // Node is called open_loop_driver
  ros::NodeHandle nh; // Main access point to communications with ROS

  // publisher for steering
  ros::Publisher steering_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", ros_buffer_size);

  // publisher for speed
  ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", ros_buffer_size);

  // subscriber to know car pose
  ros::Subscriber pose_sub = nh.subscribe("/AutoNOMOS_mini/real_pose_from_gazebo", ros_buffer_size, poseCallback);

  ros::Rate loop_rate(ros_loop_rate);

  while (ros::ok()) {
    // Set new speed for car
    ros::spinOnce(); // handles callbacks
    std_msgs::Int16 new_speed;
    new_speed.data = -100;
    speed_pub.publish(new_speed);

    // Set new steering angle for car
    std_msgs::Int16 new_steering;
    new_steering.data = 0;
    steering_pub.publish(new_steering);
    
    ROS_INFO_STREAM("Current Pose:" << current_pose);
    ROS_INFO_STREAM("Control (speed,steering): (" << new_speed << "," << new_steering << ")");

    // remaining necessary calls for ROS loop and callbacks
    loop_rate.sleep(); // waits what necessary to keep loop_rate
  }
  return 0;
}
