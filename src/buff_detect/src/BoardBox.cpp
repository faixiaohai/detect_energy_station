/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 20:43:58
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-29 19:28:29
 * @FilePath: /eigen_other/src/detect/src/BoardBox.cpp
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <detect.h>
BoardBox::BoardBox() {}
BoardBox::BoardBox(const cv::RotatedRect &rrect, const int &number, const ConditionType &type, const std::vector<cv::Point> &contour) {
    rect = rrect;
    center = rrect.center;
    height = rrect.size.height;
    width  = rrect.size.width;
    id = number;
    type_condition = type;
    contours = contour;
}
void BoardBox::ChangeId(const int &number) {
    id = number;
}

void BoardBox::ChangeCondition(const ConditionType &type) {
    type_condition = type;
}