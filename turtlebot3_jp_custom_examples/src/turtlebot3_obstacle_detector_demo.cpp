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

#include <cstdio>
#include <ctime>

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

Turtlebot3Drive::Turtlebot3Drive()
    : nh_priv_("~")
{
    // Init gazebo ros turtlebot3 node
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
    escape_range_ = 30.0 * DEG2RAD;
    check_forward_dist_ = 0.7;
    check_side_dist_ = 0.6;

    tb3_pose_ = 0.0;
    prev_tb3_pose_ = 0.0;

    // initialize subscribers
    laser_scan_sub_ = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
    odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);
    obstacle_sub_ = nh_.subscribe("obstacles", 10, &Turtlebot3Drive::obstacleMsgCallBack, this);
    cmdvel_sub_ = nh_.subscribe("cmd_vel", 10, &Turtlebot3Drive::cmdvelMsgCallBack, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 10, &Turtlebot3Drive::naviGoalCallBack, this);

    nh_.param("tf_name1", tf_name1, std::string("/base_link"));
    nh_.param("tf_name2", tf_name2, std::string("/map"));
    listener = new tf::TransformListener(ros::Duration(10));

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
    for (int i = 1; i < msg->ranges.size(); i++)
    {
        if (min_scan_ > msg->ranges.at(i))
            min_scan_ = msg->ranges.at(i);
    }
    if (publish_mode_)
    {
        fprintf(minp, "%d, %f, %f, %f, %f, %f, %f, %f", ++count, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_);
        printObstacleAngle(minp);
        fprintf(minp, "\n");
    }
}

void Turtlebot3Drive::obstacleMsgCallBack(const obstacle_detector::Obstacles::ConstPtr &msg)
{
    // ROS_INFO("obstacles msg callback");
    int i = 0;
    // ROS_INFO("robot: x:%f,y:%f", x_m, y_m);
    for (auto circle : msg->circles)
    {
        // ROS_INFO("#%d: x:%f,y:%f,vx:%f,vy:%f", ++i, circle.center.x, circle.center.y, circle.velocity.x, circle.velocity.y);
        if (hypot(circle.velocity.x, circle.velocity.y) > OBSTACLE_DETECTOR_THRESHOLD)
        {
            // ROS_INFO("#%d: %f", ++i, atan2(circle.center.y-y_m, circle.center.x-x_m)*RAD2DEG);
            double angle = (atan2(circle.center.y - y_m, circle.center.x - x_m) - th_m) * RAD2DEG;
            if (angle > 180.0)
                angle -= 360.0;
            if (angle < -180.0)
                angle += 360.0;
            obstacle_angles[i] = angle;
            obstacle_exists[i] = true;
            if (++i == 5)
                break;
        }
    }
    for (; i < 5; i++)
    {
        obstacle_exists[i] = false;
    }
}

void Turtlebot3Drive::cmdvelMsgCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_linear_ = msg->linear.x;
    cmd_vel_angular_ = msg->angular.z;
}

void Turtlebot3Drive::naviGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    publish_mode_ = true;
    // record
    char buf[32];
    std::time_t rawtime;
    std::time(&rawtime);
    std::strftime(buf, 32, "%Y-%m-%d-%H-%M-%S", std::localtime(&rawtime));

    char filename[64];
    sprintf(filename, "turtlebot3_csv_time_%s.csv", buf);
    recordp = fopen(filename, "w");
    sprintf(filename, "turtlebot3_csv_scan_%s.csv", buf);
    minp = fopen(filename, "w");
    if (recordp == nullptr || minp == nullptr)
    {
        ROS_INFO("cannnot open csv");
        ros::shutdown();
        return;
    }

    x_goal_ = msg->pose.position.x;
    y_goal_ = msg->pose.position.y;
    record_start_time_ = ros::WallTime::now();
    ROS_INFO("start recording");
    fprintf(recordp, "time, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_, o1, o2, o3, o4, o5\n");
    fprintf(minp, "step, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_, o1, o2, o3, o4, o5\n");
}

void Turtlebot3Drive::printObstacleAngle(FILE *fp)
{
    for (int i = 0; i < 5; i++)
    {
        if (obstacle_exists[i])
        {
            fprintf(fp, ", %f", obstacle_angles[i]);
        }
        else
        {
            fprintf(fp, ", ");
        }
    }
}
/*******************************************************************************
 * Control Loop function
 *******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
    static uint8_t turtlebot3_state_num = 0;
    try
    {
        ros::Time now = ros::Time::now();
        listener->waitForTransform(tf_name2, tf_name1, now, ros::Duration(5.0));
        listener->lookupTransform(tf_name2, tf_name1, now, trans_slam);
        x_m = trans_slam.getOrigin().x();
        y_m = trans_slam.getOrigin().y();
        th_m = tf::getYaw(trans_slam.getRotation());
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    if (publish_mode_)
    {
        ros::WallDuration time = ros::WallTime::now() - record_start_time_;
        moving_distance_ += hypot(x_m - prev_x_m, y_m - prev_y_m);
        prev_x_m = x_m;
        prev_y_m = y_m;
        if (recordp != nullptr)
        {
            ROS_INFO("%u.%09u, %f, %f, %f, %f, %f, %f, %f", time.sec, time.nsec, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_);
            fprintf(recordp, "%u.%09u, %f, %f, %f, %f, %f, %f, %f", time.sec, time.nsec, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_);
            printObstacleAngle(recordp);
            fprintf(recordp, "\n");
            fflush(recordp);
        }
        if (hypot(x_m - x_goal_, y_m - y_goal_) < GOAL_ERROR)
        {
            publish_mode_ = false;
            ROS_INFO("Reached to goal.");
            fclose(recordp);
            fclose(minp);
        }
    }

    return true;
}

/*******************************************************************************
 * Main function
 *******************************************************************************/
int main(int argc, char *argv[])
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
