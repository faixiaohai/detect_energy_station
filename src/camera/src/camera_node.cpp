#include <camera_node.h>

CameraCaptureNode::CameraCaptureNode(const rclcpp::NodeOptions& options)  : Node("hik_camera", options), m_camera(this->declare_parameter("camera_index", 0)) {
    //param
    m_camera_name = this->declare_parameter("camera_name", " hik_camera");
    m_camera_info_url = this->declare_parameter("camera_url", "camera/config/camera_config.yaml");
    this->declare_parameter<float>("exposuretime", 4000.0);
    this->declare_parameter<float>("gain", 15.0);
    

    // publish
    m_camera_pub = image_transport::create_camera_publisher(this, "image_raw", rmw_qos_profile_default);  // rmw_qos_profile_default 确保所有信息都发送过去 rmw_qos_profile_sensor_data  不保证所有信息都发送过去
    m_camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS());
    // service
    m_service = this->create_service<std_srvs::srv::SetBool>(
        "hik_camera_enable",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
           std::shared_ptr<std_srvs::srv::SetBool::Response> response){
            HandleCameraEnableRequest(request, response);
        });
    // camera_info_manager
    m_camera_info_manager = std::make_shared<camera_info_manager::CameraInfoManager>(this, m_camera_name);
    if (m_camera_info_manager->validateURL(m_camera_info_url)) {
        m_camera_info_manager->loadCameraInfo(m_camera_info_url);
        m_camera_info_msg = m_camera_info_manager->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid acmera info url: %s", m_camera_info_url.c_str());
    }
    auto m_thread = std::thread([this]()->void { this->PublishData(); });
}

void CameraCaptureNode::HandleCameraEnableRequest(const std::shared_ptr<std_srvs::srv::SetBool::Request> &request, std::shared_ptr<std_srvs::srv::SetBool::Response> &response) {
    m_enable = request->data;
    response->success = true;
    m_mess = m_enable ? "hik_camera_enable" : "hik_camera_disable";
    response->message = m_mess;
}

void CameraCaptureNode::InitCamera() {
    if (m_camera.OpenCamera(this->get_parameter("exposuretime").as_double())) {
        m_camera.SetGain(this->get_parameter("gain").as_double());
        // std::string camera_info = m_camera
        MV_CC_GetImageInfo(m_camera.GetHandle(), &m_camera_hik_info);
        m_image_msg.data.reserve(m_camera_hik_info.nHeightMax * m_camera_hik_info.nWidthMax * 3);
    }
}

void CameraCaptureNode::PublishData() {
    m_camera_info_pub->publish(m_camera_info);
    rclcpp::Time last_t = this->now();
    // HikFrame hik_frame;
    MV_CC_PIXEL_CONVERT_PARAM_EX m_convert_param;
    MV_FRAME_OUT out_frame;
    m_convert_param.nWidth = m_camera_hik_info.nWidthValue;
    m_convert_param.nHeight = m_camera_hik_info.nHeightValue;
    m_convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    m_camera_info_msg.header.frame_id = m_image_msg.header.frame_id =
        this->get_parameter("camera_frame").as_string();
    m_image_msg.encoding = "rgb8";
    void* handle = m_camera.GetHandle();
    while (rclcpp::ok()) {
        // check
        if (!m_enable) return;
        if (m_camera.readImageData(out_frame, 1000)) {
            // hik sdk
            m_convert_param.pDstBuffer = m_image_msg.data.data();
            m_convert_param.nDstBufferSize = m_image_msg.data.size();
            m_convert_param.pSrcData = out_frame.pBufAddr;
            m_convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
            m_convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
            MV_CC_ConvertPixelTypeEx(handle, &m_convert_param);
            // RCLCPP_INFO(this->get_logger(), "Convert Type: %f", (this->now() - s).seconds() * 1000);
            // img_msg
            m_image_msg.header.stamp = this->now();
            m_image_msg.height = out_frame.stFrameInfo.nHeight;
            m_image_msg.width = out_frame.stFrameInfo.nWidth;
            m_image_msg.step = out_frame.stFrameInfo.nWidth * 3;
            m_image_msg.data.resize(m_image_msg.width * m_image_msg.height * 3);
            // Pub
            m_camera_info_msg.header = m_image_msg.header;
            m_camera_pub.publish(m_image_msg, m_camera_info_msg);
            MV_CC_FreeImageBuffer(handle, &out_frame);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get hik out frame!");
        }
    }

}