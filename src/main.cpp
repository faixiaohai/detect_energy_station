/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 11:05:45
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-04-02 16:42:47
 * @FilePath: /eigen_other/src/main.cpp
 * @Description:  
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <iostream>
#include <detect.h>
#include <buff_pnp_solver.h>
#include <buff_tracker.h>
#include <adjustcamera.h>

int main()
{       //相机内参
    Camera camera;
    camera.OpenCamera(2000);

    std::array<double, 9> intrinsic_matrix = {2665.005527408399, 0,696.8233, 0, 2673.364537791387, 500.5147099572225,0, 0, 1};
    std::vector<double> distortion_vector = {-0.303608974614145, 4.163247825941419, -0.008432853056627, -0.003830248744148, 0};
    armor_auto_aim::BuffPnP slove_pnp(intrinsic_matrix, distortion_vector);
    Match my_match;
    ImageInformation wait_tackle_info;
    // cv::VideoCapture my_cap(PATH);
    cv::Mat frame, img_pre;
    while (1) {
        camera.GetPic(&frame);
        char key = cv::waitKey(30);
        if (key == 27) {
            break;
        }
        cv::imshow("预选", frame);
    }
    cv::destroyAllWindows();
    AdjustNumber::AdjustDynamic(frame); // 创建一个用于调节hsv上下阈值的bar。
    cv::waitKey(0); // 暂停界面用于调节阈值
    cv::RotatedRect rotatedrect = Kit::SelectRoatedRect(frame);
    RBox rbox = RBox(rotatedrect);
    my_match.now_image.rel_rbox = rbox;
    my_match.last_image.rel_rbox = rbox;
    cv::destroyAllWindows();
    BuffTracker my_tracker(0.5);
    while (1)
    {
        // my_cap.read(frame);
         camera.GetPic(&frame);
        if (frame.empty())
        {
            std::cout << "读取失败" << std::endl;
            break;
        }
        wait_tackle_info = my_match.Run(frame, "No", "draw_sloted"); // run集成运行函数 
        std::vector<cv::Point2f> points;
        cv::Mat rvec = {};
        cv::Mat tvec = {};
        if (!wait_tackle_info.wait_slot_box.empty()) {
            wait_tackle_info.wait_slot_box[0].rect.points(points);
            std::vector<cv::Point2f> corrected_points = slove_pnp.PnpSolver(wait_tackle_info.wait_slot_box[0], wait_tackle_info.rel_rbox, points, rvec, tvec);
            std::vector<cv::Point3f> world_points;
            for (const auto &point : points) {
                world_points.push_back(cv::Point3f(point.x, point.y, 0));
            } 
            world_points.push_back(cv::Point3f(wait_tackle_info.rel_rbox.center.x, wait_tackle_info.rel_rbox.center.y, 0));
            
            std::vector<cv::Point3f> camera_points = slove_pnp.AnalysisDataFromWorldToCamera(rvec, tvec);
            int b = 6;
            for (const auto &point : camera_points) {
                Eigen::MatrixXd world(3,1);
                world << point.x , point.y , point.z;
            
                cv::Point2f draw_point = slove_pnp.AnalysisDataCameraTo2D(world, rvec, tvec);    
                cv::circle(frame, draw_point, 10, cv::Scalar(0, 0, 255), 1, 8, 0);
                std::string a = std::to_string(b);
                cv::putText(frame, a, draw_point, 1, 5, cv::Scalar(255, 255, 255), 2);
                b++;
            }
            Eigen::MatrixXd pre_matrix = Eigen::MatrixXd::Zero(8, 1);
            my_tracker.Tracker(camera_points, pre_matrix);
            std::cout << pre_matrix(0);
            std::cout << pre_matrix(1);
            if (pre_matrix(1) != 0 && pre_matrix(0) != 0) {
                cv::Point2f point = slove_pnp.AnalysisDataCameraTo2D(pre_matrix);
                cv::circle(frame, point, 10, cv::Scalar(0, 255, 0), 1, 8, 0);
                int i = 1;
                for (const auto &point : corrected_points) {
                    cv::circle(frame, point, 10, cv::Scalar(0, 0, 255), 1, 8, 0);
                    std::string a = std::to_string(i);
                    cv::putText(frame, a, point, 1, 5, cv::Scalar(255, 255, 255), 2);
                    i++;
                }
                cv::imshow("pre_slot", frame);
                cv::waitKey(0);
            }
        }
    }
}