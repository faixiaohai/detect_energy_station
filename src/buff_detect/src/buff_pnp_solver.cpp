#include <buff_pnp_solver.h>

#include <vector>
#include <tuple>


#include <opencv2/core.hpp>


namespace buff_auto_aim {
BuffPnP::BuffPnP(const std::array<double, 9>& intrinsic_matrix,
                const std::vector<double>& distortion_vector)
                : m_intrinsic_matrix(cv::Mat(3, 3, CV_64F, const_cast<double*>(intrinsic_matrix.data())).clone()),
                  m_distortion_vector(cv::Mat(1, 5, CV_64F, const_cast<double*>(distortion_vector.data())).clone())
                {
                    constexpr double buff_half_x = BUFF_WIDTH / 2.0 / 1000.0;
                    constexpr double buff_half_y = BUFF_HEIGHT / 2.0 / 1000.0;
                    constexpr double buff_r = 280 / 1000.0;
                    m_buff_point3d.push_back(cv::Point3f( -buff_half_x, -buff_half_y, 0));
                    m_buff_point3d.push_back(cv::Point3f( -buff_half_x,  buff_half_y, 0));
                    m_buff_point3d.push_back(cv::Point3f(  buff_half_x,  buff_half_y, 0));
                    m_buff_point3d.push_back(cv::Point3f(  buff_half_x, -buff_half_y, 0));
                    m_buff_point3d.push_back(cv::Point3f( 0, buff_r, 0));

                } 
std::vector<cv::Point2f> BuffPnP::PnpSolver(const BoardBox& fan, const RBox &m_r_box, const std::vector<cv::Point2f>& img_points, cv::Mat& rvec, cv::Mat& tvec) {
    std::vector<cv::Point2f> points;

    points = PointOrderCorrect(img_points, m_r_box);

    cv::solvePnP(m_buff_point3d, points, m_intrinsic_matrix, m_distortion_vector,
                            rvec, tvec, false, cv::SOLVEPNP_IPPE);  // cv::SOLVEPNP_ITERATIVE ?
    m_rvec = rvec;
    m_tvec = tvec;
    return points;
}

double BuffPnP::GetDistance(const cv::Point2f& p) {
    float cx = m_intrinsic_matrix.at<double>(0, 2);
    float cy = m_intrinsic_matrix.at<double>(1, 2);
    return cv::norm(p - cv::Point2f(cx, cy));
}

std::vector<cv::Point2f> BuffPnP::PointOrderCorrect(const std::vector<cv::Point2f> &points, const RBox &m_r_box) {

    std::vector<std::tuple<cv::Point2f, int, float>> points_relative_r;
    cv::Point2f buff_box_center(0, 0); // 初始化为原点
    for (const auto &point : points) {
        cv::Point2f relative_point;
        relative_point.x = point.x - m_r_box.center.x;
        relative_point.y = m_r_box.center.y - point.y;
        float distance = std::sqrt(relative_point.x * relative_point.x + relative_point.y * relative_point.y);
        points_relative_r.emplace_back(std::make_tuple(relative_point, 0, distance));

        // 累加所有点的坐标，用于计算平均值
        buff_box_center += relative_point;
    }
    // 计算平均值
    buff_box_center /= 4;
    // 计算相对于r的坐标
    // buff_box_center -= m_r_box.center;
    std::sort(points_relative_r.begin(), points_relative_r.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a) < std::get<2>(b);
    });
    
    if (((buff_box_center.x < 0 && buff_box_center.y < 0) || (buff_box_center.x < 0 && buff_box_center.y > 0)) && std::abs(buff_box_center.y / buff_box_center.x) < 1) {
        std::get<0>(points_relative_r[0]).x;
        if (std::get<0>(points_relative_r[0]).y < std::get<0>(points_relative_r[1]).y) {
            std::get<1>(points_relative_r[0]) = 1;
            std::get<1>(points_relative_r[1]) = 2;
        } else {
            std::get<1>(points_relative_r[0]) = 2;
            std::get<1>(points_relative_r[1]) = 1;
        }
        if (std::get<0>(points_relative_r[2]).y < std::get<0>(points_relative_r[3]).y) {
            std::get<1>(points_relative_r[2]) = 0;
            std::get<1>(points_relative_r[3]) = 3;
        } else {
            std::get<1>(points_relative_r[2]) = 3;
            std::get<1>(points_relative_r[3]) = 0;
        }
    }
    if (((buff_box_center.x < 0 && buff_box_center.y > 0) || (buff_box_center.x > 0 && buff_box_center.y > 0)) && std::abs(buff_box_center.y / buff_box_center.x) > 1) {
        if (std::get<0>(points_relative_r[0]).x < std::get<0>(points_relative_r[1]).x) {
            std::get<1>(points_relative_r[0]) = 1;
            std::get<1>(points_relative_r[1]) = 2;
        } else {
            std::get<1>(points_relative_r[0]) = 2;
            std::get<1>(points_relative_r[1]) = 1;
        }
        if (std::get<0>(points_relative_r[2]).x < std::get<0>(points_relative_r[3]).x) {
            std::get<1>(points_relative_r[2]) = 0;
            std::get<1>(points_relative_r[3]) = 3;
        } else {
            std::get<1>(points_relative_r[2]) = 3;
            std::get<1>(points_relative_r[3]) = 0;
        }
    }
    if (((buff_box_center.x > 0 && buff_box_center.y < 0) || (buff_box_center.x > 0 && buff_box_center.y > 0)) && std::abs(buff_box_center.y / buff_box_center.x) < 1) {
        if (std::get<0>(points_relative_r[0]).y > std::get<0>(points_relative_r[1]).y) {
            std::get<1>(points_relative_r[0]) = 1;
            std::get<1>(points_relative_r[1]) = 2;
        } else {
            std::get<1>(points_relative_r[0]) = 2;
            std::get<1>(points_relative_r[1]) = 1;
        }
        if (std::get<0>(points_relative_r[2]).y > std::get<0>(points_relative_r[3]).y) {
            std::get<1>(points_relative_r[2]) = 0;
            std::get<1>(points_relative_r[3]) = 3;
        } else {
            std::get<1>(points_relative_r[2]) = 3;
            std::get<1>(points_relative_r[3]) = 0;
        }
    }
    if (((buff_box_center.x < 0 && buff_box_center.y < 0) || (buff_box_center.x > 0 && buff_box_center.y < 0)) && std::abs(buff_box_center.y / buff_box_center.x) > 1) {
        if (std::get<0>(points_relative_r[0]).x > std::get<0>(points_relative_r[1]).x) {
            std::get<1>(points_relative_r[0]) = 1;
            std::get<1>(points_relative_r[1]) = 2;
        } else {
            std::get<1>(points_relative_r[0]) = 2;
            std::get<1>(points_relative_r[1]) = 1;
        }
        if (std::get<0>(points_relative_r[2]).x > std::get<0>(points_relative_r[3]).x) {
            std::get<1>(points_relative_r[2]) = 0;
            std::get<1>(points_relative_r[3]) = 3;
        } else {
            std::get<1>(points_relative_r[2]) = 3;
            std::get<1>(points_relative_r[3]) = 0;
        }
    }
    std::vector<cv::Point2f> points_corrected;
    for (int i = 0; i < 4; i++) {
        for (const auto& point : points_relative_r) {
            if (std::get<1>(point) == i) {
                float a = std::get<0>(point).x + m_r_box.center.x;
                float b =  m_r_box.center.y - std::get<0>(point).y;
                points_corrected.push_back(cv::Point2f(a, b));
            }
        }   
    }
    points_corrected.push_back(cv::Point2f(m_r_box.center));
    return points_corrected;
}

cv::Point2f BuffPnP::AnalysisDataCameraTo2D(const Eigen::MatrixXd &matrix) {
    std::vector<cv::Point3f> points;
    cv::Point3f point = cv::Point3f(matrix(0), matrix(1), matrix(2));
    points.push_back(point);
    std::vector<cv::Point2f> point_2d; 
    cv::projectPoints(points, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), m_intrinsic_matrix, m_distortion_vector, point_2d);
    return point_2d[0];
}

std::vector<cv::Point3f> BuffPnP::AnalysisDataFromWorldToCamera(const cv::Mat& rvec, const cv::Mat& tvec) {
    std::vector<cv::Point3f> camera_points;
    cv::Mat temp_R;
    cv::Rodrigues(rvec, temp_R);
    Eigen::Matrix3d R = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(temp_R.ptr<double>());

    Eigen::Vector3d t;
    t << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    for (const auto &world_point : m_buff_point3d) {
        Eigen::Vector3d world_vector;
        world_vector << world_point.x, world_point.y, world_point.z;
        Eigen::Vector3d camera_vector = R * world_vector + t;
        camera_points.push_back(cv::Point3f(camera_vector(0), camera_vector(1), camera_vector(2)));
    }    
    return camera_points;
}

} // namespace armor_auto_aim