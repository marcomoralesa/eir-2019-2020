/**
 * open_loop_ocometry.cpp implements an open-loop controller for a car-like robot
 * and performs dead reckoning through numerical integration
 *
 * Subscribes to:
 *
 * Publishes to:
 *    /AutoNOMOS_mini/manual_control/steering (std_msgs/Int16)
 *    /AutoNOMOS_mini/manual_control/speed    (std_msgs/Int16)
 * Subscribes to:
 *    /AutoNOMOS_mini/real_pose_from_gazebo   (geometry_msgs/Pose2D)
 */

#include <time.h>

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>

#include "02_robot.cpp"

Robot *sim_robot, *est_robot, *imu_robot;

#define ros_buffer_size 1 // number of messages to buffer before starting to discard
#define ros_loop_rate 100 // 2Hz is the frequency of the ros loop

bool init_pose = false;
time_t current_time = time(NULL);
time_t previous_time= time(NULL);

void poseCallback(const geometry_msgs::Pose2D& msgIn) {
  sim_robot->JumpTo(msgIn.x,msgIn.y,msgIn.theta);
  if (!init_pose) {
    est_robot->JumpTo(msgIn.x,msgIn.y,msgIn.theta);
    imu_robot->JumpTo(msgIn.x,msgIn.y,msgIn.theta);
    init_pose = true;
  }
}

void imuCallback(const sensor_msgs::Imu& msgIn) {
  current_time = time(NULL);
  double delta_t = difftime(current_time, previous_time);
  previous_time = current_time;
  imu_robot->UpdateStateIMU(-msgIn.linear_acceleration.x,msgIn.linear_acceleration.y,msgIn.angular_velocity.z, delta_t);
  ROS_INFO_STREAM(msgIn);
  
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "open_loop_dead_reckoning"); // Node is called open_loop_dead_reckoning
  ros::NodeHandle nh; // Main access point to communications with ROS

  // publisher for speed
  ros::Publisher speed_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/speed", ros_buffer_size);

  // publisher for steering
  ros::Publisher steering_pub = nh.advertise<std_msgs::Int16>("/AutoNOMOS_mini/manual_control/steering", ros_buffer_size);


  // subscriber to know car pose
  ros::Subscriber pose_sub = nh.subscribe("/AutoNOMOS_mini/real_pose_from_gazebo", ros_buffer_size, poseCallback);

  // subscriber to IMU on car
  ros::Subscriber imu_sub = nh.subscribe("/AutoNOMOS_mini/imu", ros_buffer_size, imuCallback);
  
  sim_robot = new Robot(0,0,0,"Sim",&speed_pub,&steering_pub);
  est_robot = new Robot(0,0,0,"Est");  
  imu_robot = new Robot(0,0,0,"IMU");

  ros::Rate loop_rate(ros_loop_rate);
  double delta_t = 1.0/ros_loop_rate;

  double desired_v = 0.1;
  double desired_gamma = -M_PI_4/4;

  while (ros::ok()) {
    // Set new speed for car
    ros::spinOnce(); // handles callbacks

    //if (init_pose) {
      // sim robot in charge of limiting and adjusting desired parameters
      sim_robot->SetControllers(desired_v,desired_gamma);
      sim_robot->PublishControllers();

      ROS_INFO_STREAM("\n  "<<sim_robot->GetHeader()<<"\n  "<<*sim_robot);
      ROS_INFO_STREAM("\n  "<<est_robot->GetHeader()<<"\n  "<<*est_robot);
      ROS_INFO_STREAM("\n  "<<imu_robot->GetHeader()<<"\n  "<<*imu_robot);
    
      ControlU c = sim_robot->GetControllers();    
      est_robot->UpdateStateDeadReckoning(c.v,c.gamma,delta_t); //these should not be the desired values, but the actual ones used, get from sim_robot.	
      //}
    // remaining necessary calls for ROS loop and callbacks
    loop_rate.sleep(); // waits what necessary to keep loop_rate
  }
  return 0;
}
