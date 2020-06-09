#pragma once

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/calib3d.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Eigen>

typedef pcl::PointXYZ BasePoint;
typedef pcl::PointCloud<BasePoint> BaseCloud;
typedef pcl::PointXYZRGB RGBPoint;
typedef pcl::PointCloud<RGBPoint> RGBCloud;

class ProjectRGB{

public:
    ProjectRGB(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_, img_sub_, camera_info_sub_;
    ros::Publisher color_pc_pub_;

    std::string img_frame_id_, pc_frame_id_;
    
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    cv::Size image_size_;
	cv::Mat camera_instrinsics_;
	cv::Mat distortion_coefficients_;
    cv::Mat current_img_frame_;
    cv::Mat undistort_map1_, undistort_map2_;

    float fx_, fy_, cx_, cy_;
    bool get_camera_info_, get_transform_matrix_;

    Eigen::Matrix4d l_to_c_, c_to_l_;

    void points_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);
    void img_callback(const sensor_msgs::Image::ConstPtr &img_msg);
    void camera_info_callback(const sensor_msgs::CameraInfo &camera_info_msg);
};