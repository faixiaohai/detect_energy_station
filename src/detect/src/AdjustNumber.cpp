/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-17 09:51:21
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-21 19:22:36
 * @FilePath: /eigen_other/src/detect/src/AdjustNumber.cpp
 * @Description: 此文件函数用于对图像进行动态调节参数预处理，从而得到一个稳定的参数进行后面的识别
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <detect.h>
using namespace cv;
BarInformation AdjustNumber::barinformation;
AdjustNumber::AdjustNumber() {}
cv::Mat ChangeCricle(const cv::Mat &image);
cv::Mat DrawPreImg(const cv::Mat &image);
// 以下全为回调函数
static void ChangeHl(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << &image << std::endl;
    std::cout << "adjust h_l" << std::endl;
    DrawPreImg(*image);
}
void ChangeSl(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << "adjust h_s" << std::endl;
    DrawPreImg(*image);
}

void ChangeVl(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeHup(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeSup(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeVup(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeInr(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << "adjust_outr" << std::endl;
    ChangeCricle(*image);
}
void ChangeOutr(int number, void *userdata) {
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << "adjust_inr" << std::endl;
    ChangeCricle(*image);
}

/// @brief 用于生成调节hsv上下阈值的bar
/// @param image 原图像
void AdjustNumber::AdjustDynamic(cv::Mat &image) {
    std::cout << &image << std::endl;
    cv::namedWindow("self_defined", WINDOW_AUTOSIZE);
    cv::createTrackbar("h_l", "self_defined", &barinformation.h_l, 255, ChangeHl,&image);
    cv::createTrackbar("s_l", "self_defined", &barinformation.s_l, 255, ChangeSl,&image );
    cv::createTrackbar("v_l", "self_defined", &barinformation.v_l, 255, ChangeVl,&image);
    cv::createTrackbar("h_up", "self_defined", &barinformation.h_up, 255,ChangeHup,&image);
    cv::createTrackbar("s_up", "self_defined", &barinformation.s_up, 255, ChangeSup,&image);
    cv::createTrackbar("v_up", "self_defined", &barinformation.v_up, 255, ChangeVup,&image);
    cv::createTrackbar("in_r", "self_defined", &barinformation.in_r, 250, ChangeInr,&image);
    cv::createTrackbar("out_r", "self_defined", &barinformation.out_r, 500,ChangeOutr,&image);
}
/// @brief 用于对图像进行hsv处理
/// @param image 原图像
/// @return 经过hsv处理的图像
cv::Mat DrawPreImg(const cv::Mat &image) {
    cv::Mat result_img;
    ImageTrackle my_trackle;
    Match my_match;
    result_img = my_trackle.ImageTrackleHSV(image);
    cv::imshow("预调1", result_img);
    cv::waitKey(100);
    return result_img;
}
/// @brief 对图像进行hsv和内外圆分割处理
/// @param image 原图像
/// @return 返回经过内外圆处理的图像
cv::Mat ChangeCricle(const cv::Mat &image) {
    cv::Mat result_img;
    ImageTrackle my_trackle;
    Match my_match;
    result_img = my_trackle.ImageTrackleHSV(image);
    my_match.MatchRBox(result_img);
    std::cout << "rbox" << my_match.now_image.rel_box.center << std::endl;
    result_img = my_trackle.ImageTrackleCricle(result_img, my_match.now_image.rel_box);
    imshow("预调_cricle", result_img);
    cv::waitKey(100);
    return result_img;
}