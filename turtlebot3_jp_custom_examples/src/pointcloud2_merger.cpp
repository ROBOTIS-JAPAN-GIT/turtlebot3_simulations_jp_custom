
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <message_filters/subscriber.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>

#include <boost/bind/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>

class PointCloud2Merger {
private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_;

    ros::Publisher marged_cloud_pub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> tf_filter_;

    sensor_msgs::PointCloud cloud_;
    std::string output_frame_;

public:
    PointCloud2Merger() {
        nh_.param<std::string>("~output_frame", output_frame_, std::string("base_link"));
        sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "camera_right/depth/color/points", 1);
        tf_filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(sub_, tf_, output_frame_, 1));
        tf_filter_->registerCallback(boost::bind(&PointCloud2Merger::tfCloudCallback, this, boost::placeholders::_1));
        marged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("marged_cloud", 1);
    }

    ~PointCloud2Merger() {
        delete sub_;
    }

    void tfCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud) {
        sensor_msgs::PointCloud xyz_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(*cloud, xyz_cloud);

        tf_.transformPointCloud(output_frame_, xyz_cloud, cloud_);

        sensor_msgs::PointCloud output_cloud;

        output_cloud.header = cloud_.header;
        output_cloud.points.resize(cloud_.points.size());
        std::copy(cloud_.points.begin(), cloud_.points.end(), output_cloud.points.begin());
        output_cloud.channels.resize(cloud_.channels.size());
        std::copy(cloud_.channels.begin(), cloud_.channels.end(), output_cloud.channels.begin());

        marged_cloud_pub_.publish(output_cloud);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud2_merger");
    PointCloud2Merger merger;
    ros::spin();
    return 0;
}