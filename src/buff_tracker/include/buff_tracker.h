#ifndef BUFFTRACKER_H
#define BUFFTRACKER_H
#include <Kalmanfiltering.h>

#include <iostream>
#include <eigen3/Eigen/Dense>


enum BuffState {LOSTED, TRACKING, FIRSTSTART};

class BuffTracker {
public:
    
    BuffTracker(const float &d_time, const std::array<double, 9> & intrinsic_matrix, const std::vector<double>& distortion_vector);
    void Tracker(const std::vector<cv::Point3f> &points, Eigen::MatrixXd &pre_vector);

    cv::Point2f AnalysisDataCameraTo2D(const Eigen::MatrixXd &matrix);
private:
    BuffState m_state;
    cv::Mat m_intrinsic_matrix;
    cv::Mat m_distortion_vector;  
    float m_tracker_dtime;
    std::shared_ptr<BuffKalmanFilter> m_buffkalmanFilter = nullptr;
    cv::Mat m_now_tvec;
    cv::Mat m_last_tvec;
    Eigen::MatrixXd m_data;
};
#endif // BUFFTRACKER_H