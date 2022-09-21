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

#include "turtlebot3_jp_custom_examples/turtlebot3_obstacle_detector_demo.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  auto ret = init();
  ROS_ASSERT(ret);
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);
  obstacle_sub_ = nh_.subscribe("obstacles", 10, &Turtlebot3Drive::obstacleMsgCallBack, this);
  cmdvel_sub_ = nh_.subscribe("cmd_vel", 10, &Turtlebot3Drive::cmdvelMsgCallBack, this);

  nh_.param("tf_name1",tf_name1,std::string("/base_link"));
  nh_.param("tf_name2",tf_name2,std::string("/map"));
  listener = new tf::TransformListener(ros::Duration(10));

  // record
  goal_sub_ = nh_.subscribe("move_base_simple/goal", 10, &Turtlebot3Drive::naviGoalCallBack, this);
  recordp = fopen("turtlebot3_record.csv", "w");
  minp = fopen("turtlebot3_record_min.csv", "w");
  if (recordp == nullptr || minp == nullptr) {
    ROS_INFO("cannnot open csv");
    ros::shutdown();
    return false;
  }

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  static int count = 0;
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }

  // record
    min_scan_ = msg->ranges.at(0);
    for (int i = 1; i < msg->ranges.size(); i++) {
      if (min_scan_ > msg->ranges.at(i)) min_scan_ = msg->ranges.at(i);
    }
    if (publish_mode_) {
      fprintf(minp, "%d, %f, %f, %f, %f, %f, %f, %f", ++count, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_);
      fprintf(minp, "\n");
    }
}

void Turtlebot3Drive::obstacleMsgCallBack(const obstacle_detector::Obstacles::ConstPtr &msg) {
  // ROS_INFO("obstacles msg callback");
  int i = 0;
  // ROS_INFO("robot: x:%f,y:%f", x_m, y_m);
  for (auto circle : msg->circles) {
    // ROS_INFO("#%d: x:%f,y:%f,vx:%f,vy:%f", ++i, circle.center.x, circle.center.y, circle.velocity.x, circle.velocity.y);
    if (hypot(circle.velocity.x, circle.velocity.y) > 0.3) {
      // ROS_INFO("#%d: %f", ++i, atan2(circle.center.y-y_m, circle.center.x-x_m)*RAD2DEG);
    }
  }
}

void Turtlebot3Drive::cmdvelMsgCallBack(const geometry_msgs::Twist::ConstPtr &msg) {
  cmd_vel_linear_ = msg->linear.x;
  cmd_vel_angular_ = msg->angular.z;
}

void Turtlebot3Drive::naviGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  publish_mode_ = true;
  x_goal_ = msg->pose.position.x;
  y_goal_ = msg->pose.position.y;
  record_start_time_ = ros::WallTime::now();
  ROS_INFO("start recording");
}
/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;


      // SLAM
      try {
        listener->lookupTransform(tf_name2, tf_name1,ros::Time(0), trans_slam);
        x_m = trans_slam.getOrigin().x();
        y_m = trans_slam.getOrigin().y();
        th_m = tf::getYaw(trans_slam.getRotation());
      }
      catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
        // break;
      }	
      if (publish_mode_) {
        ros::WallDuration time = ros::WallTime::now() - record_start_time_;
        moving_distance_ += hypot(x_m-prev_x_m, y_m-prev_y_m);
        prev_x_m = x_m;
        prev_y_m = y_m;
        if (recordp != nullptr) {
          ROS_INFO("%u.%09u, %f, %f, %f, %f, %f, %f, %f", time.sec, time.nsec, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_);
          fprintf(recordp, "%u.%09u, %f, %f, %f, %f, %f, %f, %f", time.sec, time.nsec, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_);
          fprintf(recordp, "\n");
          fflush(recordp);
        } 
        if (hypot(x_m-x_goal_, y_m-y_goal_) < 0.1) {
          publish_mode_ = false;
          ROS_INFO("Reached to goal. Ctrl-C to shutdown.");
          fclose(recordp);
          fclose(minp);
        }
      }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
