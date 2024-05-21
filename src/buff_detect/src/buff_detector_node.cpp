#include <buff_detector_node.h>
#include <detect.h>

#include <inference.h>


namespace buff_auto_aim {

BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)  : Node("buff_detector", options) {
    DeclareParams();
    // Publisher
    m_buffs_pub = this->create_publisher<
        interfaces::msg::Buff>("buff_detector/buffs", rclcpp::SensorDataQoS());
    // Subscription
    m_cam_info_sub = this->create_subscription<
        sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(),
        std::bind(&BuffDetectorNode::SubCamInfoCallback, this, std::placeholders::_1));
    m_img_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        std::bind(&BuffDetectorNode::SubImageCallback, this, std::placeohlders::_1));
}

void BuffDetectorNode::DeclareParams() {
    auto c = this->declare_parameter("detect_color", "BLUE");
    RCLCPP_INFO(this->get_logger(), "target color: %s", c.c_str());
    auto model_path = this->declare_parameter("model_path", "/home/mayuqi/Desktop/detect_buff/model/best.onnx");
    auto inference_driver = this->declare_parameter("inference_driver", "GPU");
    m_inference = std::make_shared<OpenvinoInference>(model_path, inference_driver);

}

void BuffDetectorNode::SubCamInfoCallback(sensor_msgs::msg::CameraInfo::ConstSharedPtr cam_info) {
    m_buff_pnp = std::make_shared<BuffPnP>(cam_info->k, cam_info->d);
    m_cam_info_sub.reset();
}

void BuffDetectorNode::SubImageCallback(sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    if (m_buff_pnp == nullptr) {
        return;
    }

    cv::Mat m_img = cv_bridge::toCvShare(img_msg, "rgb8")->image;
    if (m_img.empty()) {
        return;
    }

    std::vector<InferenceResult> result;        
    m_inference->StartInfer(m_img, &result);
    if (result.size() != 0) {
        interfaces::msg::Buff buff;
        m_info = m_match.Run(m_img);
        std::vector<cv::Point2f> points;
        m_info.wait_slot_box[0].rect.points(points); 
        for (int i = 0 ; i < 4; i++) {
            buff.points[i].x = points[i].x;
            buff.points[i].y = points[i].y;
        }
        BoardBox temp_board(m_info.wait_slot_box[0].rect, 0, SLOT, m_info.wait_slot_box[0].contours);
        cv::RotatedRect demo_rect = cv::minAreaRect(result[0].points);
        RBox temp_r_box(demo_rect);
        cv::Mat r_vec;
        cv::Mat t_vec;
        std::vector<cv::Point2f> img_points;
        auto demo_vec = m_buff_pnp->PnpSolver(temp_board, temp_r_box, img_points, r_vec, t_vec );
        std::vector<cv::Point3f> camera_points = m_buff_pnp->AnalysisDataFromWorldToCamera(r_vec, t_vec);
        for (int i = 0 ; i < 4; i++) {
            buff.points[i].x = camera_points[i].x;
            buff.points[i].y = camera_points[i].y;
        }
        buff.pose.position.x = t_vec.at<double>(0);
        buff.pose.position.y = t_vec.at<double>(1);
        buff.pose.position.z = t_vec.at<double>(2);
        OrientationFromRvec(r_vec, buff.pose.orientation);
        
        m_buffs_pub->publish(buff);
    }
}

void BuffDetectorNode::OrientationFromRvec(const cv::Mat& rvec, geometry_msgs::msg::Quaternion& q) {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    tf2::Matrix3x3 tf2_rotation_matrix(
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2));
    tf2::Quaternion tf2_q;
    tf2_rotation_matrix.getRotation(tf2_q);
    q = tf2::toMsg(tf2_q);
}

} // namespace buff_auto_aim
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(buff_auto_aim::BuffDetectorNode)