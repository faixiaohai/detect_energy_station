/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 11:07:18
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-21 19:17:56
 * @FilePath: /eigen_other/src/detect/src/ImageTrackle.cpp
 * @Description: 对图片进行hsv和内外圆分割处理的函数
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <detect.h>
ImageTrackle::ImageTrackle() {
    up_thresold = cv::Scalar(AdjustNumber::barinformation.h_up,
                             AdjustNumber::barinformation.s_up,
                             AdjustNumber::barinformation.v_up);
    lower_thresold = cv::Scalar(AdjustNumber::barinformation.h_l,
                                AdjustNumber::barinformation.s_l,
                                AdjustNumber::barinformation.v_l);
    in_r = AdjustNumber::barinformation.in_r;
    out_r = AdjustNumber::barinformation.out_r;
}
/// @brief 对图像进行hsv处理 
/// @param image 要进行处理的原图片
/// @return 经过hsv二值化处理的图片
cv::Mat ImageTrackle::ImageTrackleHSV(const cv::Mat &image) {
    cv::Mat output_image, hsv_image;
    cv::cvtColor(image,hsv_image, cv::COLOR_BGR2HSV);   
    cv::inRange(hsv_image, lower_thresold, up_thresold, output_image);
    cv::Mat kernel_b = cv::getStructuringElement(0,cv::Size(2, 2));
    cv::Mat kernel_a = cv::getStructuringElement(0,cv::Size(2, 2));
    cv::morphologyEx(output_image, output_image, 1, kernel_a, cv::Point(-1, -1), 3 );
    return output_image;
} 
/// @brief 以当前帧的的R框为中心画圆，内圆用于覆盖掉流水灯，外园用于防止能量机关的待击打框被污染
/// @param image 经过hsv处理过后的图片
/// @param box 当前帧图像的R框
/// @return 经过内外圆处理后的图像
cv::Mat ImageTrackle::ImageTrackleCricle(const cv::Mat &image, const RBox &box) {
    cv::Mat out_img;
    cv::circle(image, box.center, in_r, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, box.center, out_r, cv::Scalar(0, 0, 0), 1);
    cv::Mat kernel_a = cv::getStructuringElement(0,cv::Size(3, 3));
    cv::morphologyEx(image, image, 1, kernel_a, cv::Point(-1, -1), 10 );
    return image;
}