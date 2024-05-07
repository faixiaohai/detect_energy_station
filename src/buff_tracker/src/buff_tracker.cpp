#include <buff_tracker.h>

#include <iostream>

#include <Kalmanfiltering.h>

BuffTracker::BuffTracker(const float &d_time) : m_tracker_dtime(d_time) {
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




