#include <Kalmanfiltering.h>

#include <iostream>

BuffKalmanFilter::BuffKalmanFilter(const std::vector<cv::Point3f> &points, const float &d_time) : m_dt(d_time) {
    m_mode = CLOCKWISE;
    cv::Point3f center;
    m_r_center = points[4];

    for (int i = 0; i < 4; i++) {
        center += points[i];
    }
    center /= 4;
    
    double tangent = std::abs((center.y - points[4].y) / (center.x - points[4].x));
    m_start_angle = std::abs(atan(tangent) * 180.0 / M_PI);
    center = center - points[4];
    std::cout << "releative  point" << center << std::endl;
    if (center.x < 0 && center.y < 0) {
        m_start_angle = 180 - m_start_angle;
    } else if (center.x > 0 && center.y > 0) {
        m_start_angle = 360 - m_start_angle;
    } else if (center.x < 0 && center.y > 0) {
        m_start_angle = 360 - (180 - m_start_angle);
    } else if (center.x > 0 && center.y < 0) {
        m_start_angle = m_start_angle;
    }

    if (m_mode == CLOCKWISE) {
        m_w = -w;
        m_v = v;
    } else if (m_mode == ANTICLOCKWISE) {
        m_w = w;
        m_v= -v;
    }
    last_angle = m_start_angle;
    m_angle = m_start_angle;

}

void BuffKalmanFilter::InitFilter() {
    // m_F = Eigen::MatrixXd::Zero(8, 8);

    // m_F << 1,    0,    0,   0,    m_dt, 0,    0,    0,
    //        0,    1,    0,   0,    0,    m_dt, 0,    0,
    //        0,    0,    1,   0,    0,    0,    m_dt, 0,
    //        0,    0,    0,   1,    0,    0,    0,    m_dt,
    //        0,    0,    0,   0,    1,    0,    0,    0,
    //        0,    0,    0,   0,    0,    1,    0,    0,
    //        0,    0,    0,   0,    0,    0,    1,    0,
    //        0,    0,    0,   0,    0,    0,    0,    1;

    // m_X = Eigen::MatrixXd::Zero(8, 1);
    
    // m_X << m_x, m_y, m_z, m_angle, m_Vx, m_Vy, m_Vz, m_w;

    // m_H = Eigen::MatrixXd::Zero(4, 8);
    // m_H << 1, 0, 0, 0, 0, 0, 0, 0,
    //        0, 1, 0, 0, 0, 0, 0, 0,
    //        0, 0, 1, 0, 0, 0, 0, 0,
    //        0, 0, 0, 1, 0, 0, 0, 0;
           
    //     // 定义状态向量的方差
    // double sigma_x_squared = 1; // 位置 x 的方差
    // double sigma_y_squared = 1; // 位置 y 的方差
    // double sigma_z_squared = 1;
    // double sigma_theta_squared = 0.1; // 角度的方差
    // double sigma_Vx_squared = 1; // 速度 Vx 的方差
    // double sigma_Vy_squared = 1; // 速度 Vy 的方差
    // double sigma_Vz_squared = 1; // 速度 Vz 的方差
    // double sigma_omega_squared = 0.0000001; // 角速度 omega 的方差
    // m_P0 = Eigen::MatrixXd::Zero(8, 8); 
    // // 创建初始状态协方差矩阵 
    // m_P0 << sigma_x_squared, 0, 0, 0, 0, 0, 0, 0,
    //         0, sigma_y_squared, 0, 0, 0, 0, 0, 0,
    //         0, 0, sigma_z_squared, 0, 0, 0, 0, 0,
    //         0, 0, 0, sigma_theta_squared, 0, 0, 0, 0,
    //         0, 0, 0, 0, sigma_Vx_squared, 0, 0, 0,
    //         0, 0, 0, 0, 0, sigma_Vy_squared, 0, 0,
    //         0, 0, 0, 0, 0, 0, sigma_Vz_squared, 0,
    //         0, 0, 0, 0, 0, 0, 0, sigma_omega_squared;
    // // 创建测量协方差矩阵
    // m_P1 = Eigen::MatrixXd::Zero(4, 4); 

    // m_P1 << sigma_x_squared, 0, 0, 0,
    //         0, sigma_y_squared, 0, 0,
    //         0, 0, sigma_z_squared, 0,
    //         0, 0, 0, sigma_omega_squared;
    
    // double sigma_process_x_squared = 0.1; // 位置 x 的过程噪声方差
    // double sigma_process_y_squared = 0.1; // 位置 y 的过程噪声方差
    // double sigma_process_z_squared = 0.1;
    // double sigma_process_theta_squared = 0.01; // 角度的过程噪声方差
    // double sigma_process_Vx_squared = 0.1; // 速度 Vx 的过程噪声方差
    // double sigma_process_Vy_squared = 0.1; // 速度 Vy 的过程噪声方差
    // double sigma_process_Vz_squared = 0.1; // 速度 Vz 的过程噪声方差
    // double sigma_process_omega_squared = 0.01; // 角速度 omega 的过程噪声方差

    // // 创建过程噪声协方差矩阵
    // m_Q = Eigen::MatrixXd::Zero(8, 8);
    // m_Q << sigma_process_x_squared, 0, 0, 0, 0, 0, 0, 0,
    // 0, sigma_process_y_squared, 0, 0, 0, 0, 0, 0,
    // 0, 0, sigma_process_z_squared, 0, 0, 0, 0, 0,
    // 0, 0, 0, sigma_process_theta_squared, 0, 0, 0, 0,
    // 0, 0, 0, 0, sigma_process_Vx_squared, 0, 0, 0,
    // 0, 0, 0, 0, 0, sigma_process_Vy_squared, 0, 0,
    // 0, 0, 0, 0, 0, 0, sigma_process_Vz_squared, 0,
    // 0, 0, 0, 0, 0, 0, 0, sigma_process_omega_squared; 

    // // 定义测量噪声方差
    // double sigma_measurement_x_squared = 0.1; // 位置 x 的测量噪声方差
    // double sigma_measurement_y_squared = 0.1; // 位置 y 的测量噪声方差
    // double sigma_measurement_z_squared = 0.1;   
    // double sigma_measurement_theta_squared = 0.01;

    // m_R = Eigen::MatrixXd::Zero(4, 4);
    // m_R << sigma_measurement_x_squared, 0, 0, 0,
    //     0, sigma_measurement_y_squared, 0, 0,
    //     0, 0, sigma_measurement_theta_squared, 0,
    //     0, 0, 0, sigma_measurement_z_squared;
    m_F = Eigen::MatrixXd::Zero(2, 2);
    m_F << 1, m_dt,
            0, 1;
    m_X = Eigen::MatrixXd::Zero(2, 1);
    m_X << m_angle, m_w;

    m_H = Eigen::MatrixXd::Zero(1, 2);
    m_H << 1, 0;

    m_P0 = Eigen::MatrixXd::Zero(2, 2);
    m_P0 << 0.1, 0,
            0, 0;


    m_Q = Eigen::MatrixXd::Zero(2, 2);
    m_Q << 0.1, 0,
           0, 0;

    m_R = Eigen::MatrixXd::Zero(1, 1);
    m_R << 0.1;
}


void BuffKalmanFilter::AnalysisData(const std::vector<cv::Point3f>& points) {
    cv::Point3f center;
    m_r_center = points[4];
    for (int i = 0; i < 4; i++) {
        center += points[i];
    }
    center /= 4;
    double tangent = std::abs((center.y - points[4].y) / (center.x - points[4].x));
    double angle = atan(tangent) * 180.0 / M_PI;
    center = center - points[4];
    if (center.x < 0 && center.y < 0) {
        angle = 180 - angle;
    } else if (center.x > 0 && center.y > 0) {
        angle =  360 - angle;
    } else if (center.x < 0 && center.y > 0) {
        angle = 360 - (180 - angle);
    } else if (center.x > 0 && center.y < 0) {
        angle = angle;
    }
    center = center - points[4];
    m_z = center.z;
    std::cout << "releative  point" << center << std::endl;
    // m_sensor_data = Eigen::MatrixXd::Zero(4,1);
    // m_sensor_data << center.x , center.y , center.z, angle;
    m_sensor_data = Eigen::MatrixXd::Zero(1,1);
    m_sensor_data << angle;

}



void BuffKalmanFilter::Predict() {

    m_X = m_F * m_X;
    
    m_P0 = m_F * m_P0 * m_F.transpose() + m_Q;
}

Eigen::MatrixXd BuffKalmanFilter::Update() {
    Eigen::MatrixXd K = m_P0 * m_H.transpose() * (m_H * m_P0 * m_H.transpose() + m_R).inverse();
    std::cout << "K = " << K << std::endl;
    m_X = m_X + K * (m_sensor_data - m_H * m_X);
    if (m_X(0) > 360 ) {
        m_X(0) = m_X(0) - 360;
    } else if (m_X(0) < 0) {
        m_X(0) = 360 + m_X(0);
    }

    m_P0 = (Eigen::MatrixXd::Identity(2, 2) - K * m_H) * m_P0;
    // m_P0 = (Eigen::MatrixXd::Identity(8, 8) - K * m_H) * m_P0;
    m_sensor_data = {};
    m_X_pre = Eigen::MatrixXd::Zero(2, 1);
    m_X_pre = m_F * m_X;
    if (m_X_pre(0) > 360) {
        m_X_pre(0) = m_X_pre(0) - 360;
    } else if (m_X_pre(0) < 0) {
        m_X_pre(0) = 360 + m_X_pre(0);
    }
    // std::cout << "now x : ";
    // for (int i = 0; i < 8; i++) {
    //     std::cout << m_X_pre(i) << " ";
    // }
    std::cout << "next angle : " << m_X_pre(0) << std::endl;
    std::cout << "next w : " << m_X_pre(1) << std::endl;
    std::cout << "now angle : " << m_X(0) << std::endl;
    std::cout << "now w : " << m_X(1) << std::endl;
    std::cout <<  "===========================================================" << std::endl;
    Eigen::MatrixXd  x_pre(5,1);
    // 将角度转换为弧度
    float angle_rad = m_X_pre(0) * M_PI / 180.0f;
    // 计算cos和sin
    float pre_X = std::cos(angle_rad);
    float pre_Y = std::sin(angle_rad);
    last_angle = m_X_pre(0);
    x_pre <<  -(pre_X * R  + m_r_center.x), -(-(pre_Y * R) + m_r_center.y), m_z , m_X_pre(0), 1;
    return x_pre;
}
