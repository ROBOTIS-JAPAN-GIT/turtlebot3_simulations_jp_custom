#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "turtlebot3_trajectory_publisher");

    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ros::Publisher trajectory_pub = n.advertise<nav_msgs::Path>("tb3_trajectory", 1000);

    tf::TransformListener *listener;
    tf::StampedTransform trans_slam;
    std::string tf_base_link;
    std::string tf_map;

    n.param("tf_base_link", tf_base_link, std::string("/base_link"));
    n.param("tf_map", tf_map, std::string("/map"));
    listener = new tf::TransformListener(ros::Duration(10));

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        
        try {
        listener->waitForTransform(tf_map, tf_base_link, now, ros::Duration(5.0));
        listener->lookupTransform(tf_map, tf_base_link, now, trans_slam);
        } 
        catch (tf::TransformException &ex)  {
            ROS_ERROR("%s", ex.what());
        }	

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";
        pose.pose.position.x = trans_slam.getOrigin().x();
        pose.pose.position.y = trans_slam.getOrigin().y();
        pose.pose.orientation.z = tf::getYaw(trans_slam.getRotation());

        path.poses.push_back(pose);
        trajectory_pub.publish(path);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}