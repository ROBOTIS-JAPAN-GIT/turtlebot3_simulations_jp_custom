#include <cstdio>
#include <ctime>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <obstacle_detector/Obstacles.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)


#define GOAL_ERROR 0.1
#define OBSTACLE_DETECTOR_THRESHOLD 0.1

class Turtlebot3CSVRecorder
{
 public:
  Turtlebot3CSVRecorder();
  ~Turtlebot3CSVRecorder();
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
  ros::Subscriber waypoint_goal_sub_;
  ros::Subscriber waypoint_sub_;

  // tf
  tf::TransformListener *listener;
  tf::StampedTransform trans_slam;
  std::string tf_base_link;
  std::string tf_map;
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

  double cmd_vel_linear_;
  double cmd_vel_angular_;

  double moving_distance_ = 0.0;
  int count = 0;

  // Function prototypes
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void obstacleMsgCallBack(const obstacle_detector::Obstacles::ConstPtr &msg);
  void cmdvelMsgCallBack(const geometry_msgs::Twist::ConstPtr &msg);
  void waypointGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void waypointCallBack(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg);
  
  // obstacle_detector
  double obstacle_angles[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  double obstacle_exists[5] = {false, false, false, false, false};
  void printObstacleAngle(FILE *fp);
  void startRecord(double goal_x, double goal_y);
};

Turtlebot3CSVRecorder::Turtlebot3CSVRecorder()
    : nh_priv_("~")
{
    std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

    laser_scan_sub_ = nh_.subscribe("scan", 10, &Turtlebot3CSVRecorder::laserScanMsgCallBack, this);
    obstacle_sub_ = nh_.subscribe("obstacles", 10, &Turtlebot3CSVRecorder::obstacleMsgCallBack, this);
    cmdvel_sub_ = nh_.subscribe("cmd_vel", 10, &Turtlebot3CSVRecorder::cmdvelMsgCallBack, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 10, &Turtlebot3CSVRecorder::naviGoalCallBack, this);
    waypoint_sub_= nh_.subscribe("move_base/goal", 10, &Turtlebot3CSVRecorder::waypointCallBack, this);
    waypoint_goal_sub_= nh_.subscribe("turtlebot3/waypoint_goal", 10, &Turtlebot3CSVRecorder::waypointGoalCallBack, this);

    nh_.param("tf_base_link", tf_base_link, std::string("/base_link"));
    nh_.param("tf_map", tf_map, std::string("/map"));
    listener = new tf::TransformListener(ros::Duration(10));
}

Turtlebot3CSVRecorder::~Turtlebot3CSVRecorder()
{
    ros::shutdown();
}



void Turtlebot3CSVRecorder::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
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

void Turtlebot3CSVRecorder::obstacleMsgCallBack(const obstacle_detector::Obstacles::ConstPtr &msg)
{
    int i = 0;
    for (auto circle : msg->circles)
    {
        if (hypot(circle.velocity.x, circle.velocity.y) > OBSTACLE_DETECTOR_THRESHOLD)
        {
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

void Turtlebot3CSVRecorder::cmdvelMsgCallBack(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_linear_ = msg->linear.x;
    cmd_vel_angular_ = msg->angular.z;
}

void Turtlebot3CSVRecorder::naviGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (publish_mode_ == true) return;
    startRecord(msg->pose.position.x,  msg->pose.position.y);
}

void Turtlebot3CSVRecorder::waypointGoalCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (publish_mode_ == true) return;
    startRecord(msg->pose.position.x,  msg->pose.position.y);
}

void Turtlebot3CSVRecorder::waypointCallBack(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg) {
    ROS_INFO("waypoint: x=%f,y=%f", msg->goal.target_pose.pose.position.x, msg->goal.target_pose.pose.position.y);
}

void Turtlebot3CSVRecorder::startRecord(double goal_x, double goal_y) {
    publish_mode_ = true;
    count = 0;

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

    x_goal_ = goal_x;
    y_goal_ = goal_y;
    record_start_time_ = ros::WallTime::now();
    ROS_INFO("start recording");
    fprintf(recordp, "time, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_, o1, o2, o3, o4, o5\n");
    fprintf(minp, "step, x_m, y_m, th_m, cmd_vel_linear_, cmd_vel_angular_, moving_distance_, min_scan_, o1, o2, o3, o4, o5\n");
}


void Turtlebot3CSVRecorder::printObstacleAngle(FILE *fp)
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

bool Turtlebot3CSVRecorder::controlLoop()
{
    try
    {
        ros::Time now = ros::Time::now();
        listener->waitForTransform(tf_map, tf_base_link, now, ros::Duration(5.0));
        listener->lookupTransform(tf_map, tf_base_link, now, trans_slam);
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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtlebot3_csv_recorder");
    Turtlebot3CSVRecorder csv_recorder;

    ros::Rate loop_rate(125);

    while (ros::ok())
    {
        csv_recorder.controlLoop();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
