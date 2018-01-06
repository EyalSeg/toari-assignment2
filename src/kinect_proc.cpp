#include "ros/ros.h"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv


void onNewDepth(const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::fromROSMsg (*input, cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudp (new pcl::PointCloud<pcl::PointXYZRGBA> (cloud));

    if (cloudp->empty()) {

        ROS_WARN("empty cloud");
        return;
    }

    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*input, *image_msg);
    image_msg->header.stamp = input->header.stamp;
    image_msg->header.frame_id = input->header.frame_id;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    Mat result=cv_ptr->image;
}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "kinect_proc");
     ros::NodeHandle n;
     ros::Subscriber sub = n.subscribe("/kinect2/qhd/points", 1, onNewDepth);

}