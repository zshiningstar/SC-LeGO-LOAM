#pragma once

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Eigen>

typedef pcl::PointXYZ BasePoint;
typedef pcl::PointCloud<BasePoint> BaseCloud;

class ProjectRGB{

public:
    ProjectRGB(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_, img_sub_, camera_info_sub_;

    std::string img_frame_id_, pc_frame_id_;
    
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    cv::Size image_size_;
	cv::Mat camera_instrinsics_;
	cv::Mat distortion_coefficients_;
    float fx_, fy_, cx_, cy_;

    void points_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
    void img_callback(const sensor_msgs::Image::ConstPtr &img_msg);
    void camera_info_callback(const sensor_msgs::CameraInfo &camera_info_msg);
};