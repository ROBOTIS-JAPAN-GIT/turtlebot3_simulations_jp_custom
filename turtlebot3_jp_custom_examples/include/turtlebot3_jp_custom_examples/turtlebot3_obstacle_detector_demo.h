/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <obstacle_detector/Obstacles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TB3_DIRECTION 0
#define TB3_DRIVE_FORWARD 1
#define TB3_RIGHT_TURN    2
#define TB3_LEFT_TURN     3

#define GOAL_ERROR 0.1
#define OBSTACLE_DETECTOR_THRESHOLD 0.1

class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber obstacle_sub_;
  ros::Subscriber cmdvel_sub_;

  // tf
  tf::TransformListener *listener;
  tf::StampedTransform trans_slam;
  std::string tf_name1;
  std::string tf_name2;
  double x_m=0.0, y_m=0.0, th_m=0.0;
  double prev_x_m=0.0, prev_y_m=0.0;

  // record
  ros::Subscriber goal_sub_;
  bool publish_mode_ = false;
  void naviGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
  double x_goal_ = 0.0;
  double y_goal_ = 0.0;

  double min_scan_;

  ros::WallTime record_start_time_;
  FILE *recordp;
  FILE *minp;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};

  double tb3_pose_;
  double prev_tb3_pose_;

  double cmd_vel_linear_;
  double cmd_vel_angular_;

  double moving_distance_ = 0.0;

  // Function prototypes
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void obstacleMsgCallBack(const obstacle_detector::Obstacles::ConstPtr &msg);
  void cmdvelMsgCallBack(const geometry_msgs::Twist::ConstPtr &msg);
  
  // obstacle_detector
  double obstacle_angles[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double obstacle_exists[5] = {false, false, false, false, false};
  void printObstacleAngle(FILE *fp);
};
#endif // TURTLEBOT3_DRIVE_H_
