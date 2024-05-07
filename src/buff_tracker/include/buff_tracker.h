#ifndef BUFFTRACKER_H
#define BUFFTRACKER_H
#include <Kalmanfiltering.h>

#include <iostream>
#include <eigen3/Eigen/Dense>

#include <detect.h>

enum BuffState {LOSTED, TRACKING, FIRSTSTART};

class BuffTracker {
public:
    
    BuffTracker(const float &d_time);
    void Tracker(const std::vector<cv::Point3f> &points, Eigen::MatrixXd &pre_vector);
private:
    BuffState m_state;
    float m_tracker_dtime;
    std::shared_ptr<BuffKalmanFilter> m_buffkalmanFilter = nullptr;
    cv::Mat m_now_tvec;
    cv::Mat m_last_tvec;
    Eigen::MatrixXd m_data;
};
#endif // BUFFTRACKER_H