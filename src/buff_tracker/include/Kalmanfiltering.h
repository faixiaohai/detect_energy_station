#ifndef KALMANFILTERING_H
#define KALMANFILTERING_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>


enum BuffRotateMode {CLOCKWISE, ANTICLOCKWISE};

class BuffKalmanFilter 
{
    static constexpr float R = 280 / 1000.0;
    static constexpr float v = 0.73;
    static constexpr float w = 60;
public:
    BuffKalmanFilter(const std::vector<cv::Point3f> &points , const float &d_time);


    void InitFilter();


    void AnalysisData(const std::vector<cv::Point3f> &points);
    
    void Predict();

    Eigen::MatrixXd Update();

public:
    float last_angle;
    float m_dt;
    float m_w; // 恒
    Eigen::MatrixXd m_sensor_data; // 观测向量
private:
    BuffRotateMode m_mode;
    float m_v;
    float m_r;
    float m_start_angle;
    float m_z;
    float m_angle;
    cv::Point3f m_r_center;
    Eigen::MatrixXd m_F; // 状态转移矩阵
    Eigen::MatrixXd m_H; // 观测矩阵
    Eigen::MatrixXd m_Q; // 协方差矩阵
    Eigen::MatrixXd m_R; // 观测方差矩阵
    Eigen::MatrixXd m_P0; // 初始状态协方差矩阵
    Eigen::MatrixXd m_P1; // 初始观测协方差矩阵
    Eigen::MatrixXd m_X; // 状态向量
    Eigen::MatrixXd m_X_pre; //
    
};

#endif // KALMANFILTERING_H


