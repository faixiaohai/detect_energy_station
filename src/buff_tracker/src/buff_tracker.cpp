#include <buff_tracker.h>

#include <iostream>

#include <Kalmanfiltering.h>

BuffTracker::BuffTracker(const float &d_time,
                        const std::array<double, 9>& intrinsic_matrix,
                        const std::vector<double>& distortion_vector)
                        : m_intrinsic_matrix(cv::Mat(3, 3, CV_64F, const_cast<double*>(intrinsic_matrix.data())).clone()),
                        m_distortion_vector(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_vector.data())).clone()) {

    m_tracker_dtime = d_time;
    m_state = FIRSTSTART;
}
void BuffTracker::Tracker(const std::vector<cv::Point3f> &points, Eigen::MatrixXd &pre_vector) {

    if (m_state == FIRSTSTART) {
        BuffKalmanFilter buff_kalman_filter(points, m_tracker_dtime);
        m_buffkalmanFilter = std::make_shared<BuffKalmanFilter>(buff_kalman_filter);
        m_buffkalmanFilter->InitFilter();
        m_state = TRACKING;
        return;
    }
    if (m_state == TRACKING) {
        m_buffkalmanFilter->AnalysisData(points); 
        if (std::abs(m_buffkalmanFilter->last_angle - m_buffkalmanFilter->m_sensor_data(0)) > std::abs((m_buffkalmanFilter->m_dt * m_buffkalmanFilter-> m_w * 2))) {
            m_buffkalmanFilter->last_angle = pre_vector(3);
            m_buffkalmanFilter = nullptr;
            m_state = FIRSTSTART;
            return;
        }
        m_buffkalmanFilter->Predict();
        pre_vector = m_buffkalmanFilter->Update();        
        m_buffkalmanFilter->m_sensor_data = {};
        m_buffkalmanFilter->last_angle = pre_vector(3);
    } else if (m_state == LOSTED) {
        m_buffkalmanFilter = nullptr;
    }   
}

cv::Point2f BuffTracker::AnalysisDataCameraTo2D(const Eigen::MatrixXd &matrix) {
    std::vector<cv::Point3f> points;
    cv::Point3f point = cv::Point3f(matrix(0), matrix(1), matrix(2));
    points.push_back(point);
    std::vector<cv::Point2f> point_2d; 
    cv::projectPoints(points, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), m_intrinsic_matrix, m_distortion_vector, point_2d);
    return point_2d[0];
}




