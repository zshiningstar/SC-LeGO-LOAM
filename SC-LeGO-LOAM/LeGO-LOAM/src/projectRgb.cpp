#include "projectRgb.h"

ProjectRGB::ProjectRGB(ros::NodeHandle &nh):nh_(nh), tf2_listener_(tf2_buffer_){

    std::string points_topic = nh_.param<std::string>("points_topic", "/velodyne_points");
    std::string img_topic = nh_.param<std::string>("image_topic", "/image_raw");
    std::string camera_info_topic = nh_.param<std::string>("camera_info_topic", "/camera_info");

    img_frame_id_ = pc_frame_id_ = "";

    pc_sub_ = nh_.subscribe(points_topic, 10, &ProjectRGB::points_callback, this);
    img_sub_ = nh_.subscribe(img_topic, 10, &ProjectRGB::img_callback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &ProjectRGB::camera_info_callback, this);

}

void ProjectRGB::points_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg){
    BaseCloud::Ptr input_cloud_ptr(new BaseCloud);

    pcl::fromROSMsg(*pc_msg, *input_cloud_ptr);
    pc_frame_id_ = pc_msg->header.frame_id;

    if ( (img_frame_id_!="") && (pc_frame_id_ !=""))
    {
        try
        {
            tf2_buffer_.lookupTransform(img_frame_id_, pc_frame_id_, ros::Time(0));
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}

void ProjectRGB::img_callback(const sensor_msgs::Image::ConstPtr &img_msg){
    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    cv::Mat img_mat = cv_image_ptr->image;
    img_frame_id_ = img_msg->header.frame_id;
    
    // debug
    // cv::namedWindow("debug", cv::WINDOW_NORMAL);
    // cv::imshow("debug", img_mat);
    // cv::waitKey(5);
    
}

void ProjectRGB::camera_info_callback(const sensor_msgs::CameraInfo &camera_info_msg){
	image_size_.height = camera_info_msg.height;
	image_size_.width = camera_info_msg.width;

	camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			camera_instrinsics_.at<double>(row, col) = camera_info_msg.K[row * 3 + col];
		}
	}

	distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
	for (int col = 0; col < 5; col++)
	{
		distortion_coefficients_.at<double>(col) = camera_info_msg.D[col];
	}

	fx_ = static_cast<float>(camera_info_msg.P[0]);
	fy_ = static_cast<float>(camera_info_msg.P[5]);
	cx_ = static_cast<float>(camera_info_msg.P[2]);
	cy_ = static_cast<float>(camera_info_msg.P[6]);
    
	ROS_INFO("[%s] CameraIntrinsics obtained.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "project_rgb");

    ros::NodeHandle nh("~");

    ProjectRGB projecter(nh);
    ros::spin();

    return 0;
}
