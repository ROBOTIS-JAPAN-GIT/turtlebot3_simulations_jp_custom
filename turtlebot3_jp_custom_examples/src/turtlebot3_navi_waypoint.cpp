#include <vector>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>

int main(int argc, char* argv[])
{
    std::vector<std::vector<double>> waypoints = {
    //  {x  , y     , yaw   },
        {0  , -2.5  , -1.7  },
        {3  , -2.5  , -0    },
        {3  , +2.5  ,  1.7  },
        {0  , +2.5  , -3.14 }
    };

    ros::init(argc, argv, "turtlebot3_navi_waypoint");

    ros::NodeHandle n;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action("move_base", true);
    while (! move_base_action.waitForServer(ros::Duration(60.0)));
    ROS_INFO("action server ok");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    ros::Publisher final_goal_pub = n.advertise<geometry_msgs::PoseStamped>("turtlebot3/waypoint_goal", 1000);
    geometry_msgs::PoseStamped final_goal;
    final_goal.header.frame_id = "map";
    final_goal.header.stamp = ros::Time::now();
    final_goal.pose.position.x = waypoints.back().at(0);
    final_goal.pose.position.y = waypoints.back().at(1);
    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0.0, 0.0, waypoints.back().at(2)), final_goal.pose.orientation);
    ros::Duration(0.5).sleep();
    final_goal_pub.publish(final_goal);

    int i = 0;
    for (std::vector<double> waypoint : waypoints) {
        if (! ros::ok()) break;
        goal.target_pose.pose.position.x = waypoint.at(0);
        goal.target_pose.pose.position.y = waypoint.at(1);
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoint.at(2));
        move_base_action.sendGoal(goal);
        ROS_INFO("sent goal no.%d", ++i);

        bool succeed = move_base_action.waitForResult(ros::Duration(60.0));
        actionlib::SimpleClientGoalState state = move_base_action.getState();
        ROS_INFO("%s. (%s)", succeed ? "Suceed" : "Fail", state.toString().c_str());
    }
    return 0;
}