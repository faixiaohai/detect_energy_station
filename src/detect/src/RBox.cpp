/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 16:20:23
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-28 18:48:49
 * @FilePath: /eigen_other/src/detect/src/RBox.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <detect.h>
RBox::RBox() {}
RBox::RBox(const cv::RotatedRect &rect) {
    box_rect = rect;
    center = rect.center;
    height = rect.size.height;
    width = rect.size.width;
}