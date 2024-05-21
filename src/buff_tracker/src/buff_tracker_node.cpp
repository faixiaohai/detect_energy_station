#include <buff_tracker_node.h>

namespace buff_auto_aim {

BuffTrackerNode::BuffTrackerNode(const rclcpp::NodeOptions &options) : Node("buff_tracker", options) {
    m_buff_sub = this->create_subscription<interfaces::msg::Buff>("buff_detector/buffs", rclcpp::SensorDataQoS(), std::bind(&BuffTrackerNode::SubDetectorInfoCallback, this, std::placeholders::_1));
    m_camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(),std::bind(&BuffTrackerNode::SubCameraInfoCallback, this, std::placeholders::_1));
} 

void BuffTrackerNode::SubDetectorInfoCallback(const interfaces::msg::Buff &buff_msg) {
    std::vector<cv::Point3f> camera_points;
    for (auto &point : buff_msg.points) {
        camera_points.push_back(cv::Point3f(point.x, point.y, point.z));
    }
    Eigen::MatrixXd pre_matrix = Eigen::MatrixXd::Zero(8, 1);
    m_tracker->Tracker(camera_points,pre_matrix);
    cv::Point2f point_2d = m_tracker->AnalysisDataCameraTo2D(pre_matrix);
}



void  BuffTrackerNode::SubCameraInfoCallback(const  sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_msg) {
    auto m_t = this->declare_parameter("m_t", 50);
    m_tracker = std::make_shared<BuffTracker>(m_t, m_intrinsic_matrix, m_distortion_vector);
    m_distortion_vector = m_distortion_vector(cv::Mat(1, 5, CV_64F, const_cast<double*>(camera_msg->d.data())).clone());
    m_intrinsic_matrix = (cv::Mat(3, 3, CV_64F, const_cast<double*>(camera_msg->k.data())).clone());
}

} // namespace buff_auto_aim