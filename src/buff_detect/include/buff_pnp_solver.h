#ifndef ARMOR_AUTO_AIM_BUFF_DETECTOR_BUFF_PNP_SOLVER_H
#define ARMOR_AUTO_AIM_BUFF_DETECTOR_BUFF_PNP_SOLVER_H

#include <detect.h>

#include <vector>


#include <eigen3/Eigen/Core>




namespace buff_auto_aim {
class BuffPnP {
public:
    BuffPnP(const std::array<double, 9>& intrinsic_matrix,
            const std::vector<double>& distortion_vector); 
    std::vector<cv::Point2f> PnpSolver(const BoardBox& fan, const RBox& m_r_box, const std::vector<cv::Point2f>& img_points, cv::Mat& rvec, cv::Mat& tvec);

    double GetDistance(const cv::Point2f& p);

    std::vector<cv::Point3f> AnalysisDataFromWorldToCamera(const cv::Mat& rvec, const cv::Mat& tvec);

    cv::Point2f AnalysisDataCameraTo2D(const Eigen::MatrixXd & matrix);
    
    std::vector<cv::Point2f> PointOrderCorrect(const std::vector<cv::Point2f>& points, const  RBox &m_r_box);
private:
    static constexpr float BUFF_WIDTH = 160;
    static constexpr float BUFF_HEIGHT = 180;
    cv::Mat m_rvec;
    cv::Mat m_tvec;
    cv::Mat m_intrinsic_matrix;
    cv::Mat m_distortion_vector;
    std::vector<cv::Point3d> m_buff_point3d;
    std::shared_ptr<cv::RotatedRect> m_r_box = std::make_shared<cv::RotatedRect>();
};
}

#endif
