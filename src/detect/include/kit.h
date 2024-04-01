/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-30 13:35:29
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-31 17:26:41
 * @FilePath: /eigen_other/src/detect/include/kit.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <iostream>
#include <opencv2/opencv.hpp>

namespace Kit
{
    float ComputeDistance(const cv::Point2f &pt1, const cv::Point2f &pt2);
    cv::RotatedRect SelectRoatedRect(const cv::Mat &image);
    float ComputeIou(const cv::RotatedRect &rect_one, const cv::RotatedRect &rect_two);
    float ComputeIou(const cv::Rect &rect_one, const cv::Rect &rect_two);
    float CIou(const cv::RotatedRect &rect_one, const cv::RotatedRect &rect_two);
} // namespace kit