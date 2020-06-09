#include "projectRgb.h"

ProjectRGB::ProjectRGB(ros::NodeHandle &nh):nh_(nh), tf2_listener_(tf2_buffer_){

    std::string points_topic = nh_.param<std::string>("points_topic", "/velodyne_points");
    std::string img_topic = nh_.param<std::string>("image_topic", "/image_raw");
    std::string camera_info_topic = nh_.param<std::string>("camera_info_topic", "/camera_info");
    std::string out_points_topic = nh_.param<std::string>("out_points_topic", "/colored_points");

    img_frame_id_ = pc_frame_id_ = "";
    get_camera_info_ = false;
    get_transform_matrix_ = false;

    pc_sub_ = nh_.subscribe(points_topic, 10, &ProjectRGB::points_callback, this);
    img_sub_ = nh_.subscribe(img_topic, 10, &ProjectRGB::img_callback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &ProjectRGB::camera_info_callback, this);

    color_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(out_points_topic, 10);

}

void ProjectRGB::points_callback(const sensor_msgs::PointCloud2ConstPtr &pc_msg){

    pc_frame_id_ = pc_msg->header.frame_id;

    if ( (img_frame_id_=="") || (pc_frame_id_ =="")){
        return;
    }
    BaseCloud::Ptr input_cloud_ptr(new BaseCloud);
    RGBCloud::Ptr output_cloud_ptr(new RGBCloud);

    pcl::fromROSMsg(*pc_msg, *input_cloud_ptr);

    if (!get_transform_matrix_)
    {
        geometry_msgs::TransformStamped lidar_to_cam_tf;
        try{
            lidar_to_cam_tf = tf2_buffer_.lookupTransform(img_frame_id_, 
                                                        pc_frame_id_, ros::Time(0));
        }
        catch(const std::exception& e){
            std::cerr << e.what() << '\n';
        }
        l_to_c_ = tf2::transformToEigen(lidar_to_cam_tf).matrix();
        c_to_l_ = l_to_c_.inverse();
        get_transform_matrix_ = true;
    }
    
    BaseCloud::Ptr pc_in_camera(new BaseCloud);
    
    pcl::transformPointCloud(*input_cloud_ptr, *pc_in_camera, l_to_c_);
#pragma omp for
    for (size_t i = 0; i < pc_in_camera->points.size(); i++)
	{
		int col = int(pc_in_camera->points[i].x * fx_ / pc_in_camera->points[i].z + cx_);
		int row = int(pc_in_camera->points[i].y * fy_ / pc_in_camera->points[i].z + cy_);
        RGBPoint colored_3d_point;
		if ((col >= 0) && (col < image_size_.width)
			&& (row >= 0) && (row < image_size_.height)
			&& pc_in_camera->points[i].z > 0)
		{
            cv::Vec3b rgb_pixel = current_img_frame_.at<cv::Vec3b>(row, col);
            colored_3d_point.x = pc_in_camera->points[i].x;
            colored_3d_point.y = pc_in_camera->points[i].y;
            colored_3d_point.z = pc_in_camera->points[i].z;
            colored_3d_point.r = rgb_pixel[2];
            colored_3d_point.g = rgb_pixel[1];
            colored_3d_point.b = rgb_pixel[0];

			output_cloud_ptr->points.push_back(colored_3d_point);
		}else{
            colored_3d_point.x = pc_in_camera->points[i].x;
            colored_3d_point.y = pc_in_camera->points[i].y;
            colored_3d_point.z = pc_in_camera->points[i].z;
            colored_3d_point.r = 0;
            colored_3d_point.g = 0;
            colored_3d_point.b = 0;

            output_cloud_ptr->points.push_back(colored_3d_point);
        }
	}

    RGBCloud::Ptr final_output_pc_ptr(new RGBCloud);
    pcl::transformPointCloud(*output_cloud_ptr, *final_output_pc_ptr, c_to_l_);

    sensor_msgs::PointCloud2 output_pc_msg;
    pcl::toROSMsg(*final_output_pc_ptr, output_pc_msg);
    output_pc_msg.header = pc_msg->header;

    color_pc_pub_.publish(output_pc_msg);
    
}

void ProjectRGB::img_callback(const sensor_msgs::Image::ConstPtr &img_msg){

    if(!get_camera_info_){
        return;
    }

    cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
    
    current_img_frame_ = cv_image_ptr->image;
    img_frame_id_ = img_msg->header.frame_id;

    // cv::undistort(img_mat, current_img_frame_, camera_instrinsics_, distortion_coefficients_);
    // cv::remap(img_mat, current_img_frame_, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
    image_size_.width = current_img_frame_.cols;
    image_size_.height = current_img_frame_.rows;

    // debug
    // cv::namedWindow("debug", cv::WINDOW_NORMAL);
    // cv::imshow("debug", current_img_frame_);
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

    get_camera_info_ = true;

    // load image undistort params and get the re-map param
    // 去畸变并保留最大图
    cv::initUndistortRectifyMap(camera_instrinsics_, distortion_coefficients_, cv::Mat(), 
            cv::getOptimalNewCameraMatrix(camera_instrinsics_,distortion_coefficients_,image_size_, 1, image_size_, 0), 
            image_size_, CV_16SC2,undistort_map1_, undistort_map2_);

    camera_info_sub_.shutdown();
	ROS_INFO("CameraIntrinsics obtained.");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "project_rgb");

    ros::NodeHandle nh("~");

    ProjectRGB projecter(nh);
    ros::spin();

    return 0;
}
