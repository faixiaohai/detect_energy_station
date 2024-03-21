/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 11:07:18
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-21 19:52:00
 * @FilePath: /eigen_other/src/detect/src/Match.cpp
 * @Description: 用于待打击框选取的一系列函数
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <detect.h>
Match::Match() {}
/// @brief 对输入的图像进行寻找轮廓，框选轮廓，和上一帧的R框进行iou操作从而选定当前帧的R框，如果上一帧并没有R框，则计算所有轮廓到图像中心的距离，距离最短的为R框;
/// @param image 经过hsv二值化处理的图像 
void Match::MatchRBox(const cv::Mat &image) {
    std::vector<std::vector<cv::Point>> contours; // 存放临时轮廓的vector向量
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, 0, 2);
    now_image.contours = contours;
    cv::RotatedRect rotated_rect;
    for (auto &contour : contours ) {
        if (cv::contourArea(contour) > RBOX_MAX_AREAM) {
            continue;
        }
        rotated_rect = cv::minAreaRect(contour);
        RBox rbox(rotated_rect);
        now_image.pre_rbox.push_back(rbox);
    }
    std::vector<float> iou_arry;
    if (last_image.rel_box.center.x != 0) {
        for (int i = 0; i < now_image.pre_rbox.size(); i++) {
            float iou = ComputeIou(last_image.rel_box, now_image.pre_rbox[i]);
            iou_arry.push_back(iou);
        }
        auto it = std::max_element(iou_arry.begin(), iou_arry.end());
        if (it != iou_arry.end()) {
            int index = std::distance(iou_arry.begin(), it);
            now_image.rel_box = now_image.pre_rbox[index];
        }       
    } else {
        cv::Point2f image_center;
        image_center.x = image.size().width / 2;
        image_center.y = image.size().height / 2;
        std::cout << "image center = " << image_center << std::endl;
        std::vector<float> distances;
        for (int i = 0; i < now_image.pre_rbox.size(); i++) {
            float distance = std::sqrt(abs(image_center.x - now_image.pre_rbox[i].center.x) * abs(image_center.x - now_image.pre_rbox[i].center.x) + abs(image_center.y -now_image.pre_rbox[i].center.y) * abs(image_center.y -now_image.pre_rbox[i].center.y));
            distances.push_back(distance);
        }
        auto min = std::min_element(distances.begin(), distances.end());
        if (min != distances.end()) {
            int index = std::distance(distances.begin(), min);
            now_image.rel_box = now_image.pre_rbox[index];
            std::cout << "founed rbox in now image" << std::endl;
        }
    }
    iou_arry.clear();
}

/// @brief 计算iou值的函数
/// @param rect_one 第一个框
/// @param rect_two 第二个框
/// @return iou值
float Match::ComputeIou(const RBox &rect_one, const RBox &rect_two) {
    // 计算交集部分
    int x1 = std::max(rect_one.center.x, rect_two.center.x);
    int y1 = std::max(rect_one.center.y, rect_two.center.y);
    int x2 = std::min(rect_one.center.x + rect_one.width, rect_two.center.x + rect_two.width);
    int y2 = std::min(rect_one.center.y + rect_one.height, rect_two.center.y + rect_two.height);

    // 计算交集和并集的面积
    int intersection_area = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int box1_area = rect_one.width * rect_one.height;
    int rect_two_area = rect_two.width * rect_two.height;
    int union_area = box1_area + rect_two_area - intersection_area;

    // 计算IOU值
    float iou = static_cast<float>(intersection_area) / union_area;
    return iou;
}
float Match::ComputeIou(const BoardBox &rect_one, const BoardBox &rect_two) {
    // 计算交集部分
    int x1 = std::max(rect_one.center.x, rect_two.center.x);
    int y1 = std::max(rect_one.center.y, rect_two.center.y);
    int x2 = std::min(rect_one.center.x + rect_one.width, rect_two.center.x + rect_two.width);
    int y2 = std::min(rect_one.center.y + rect_one.height, rect_two.center.y + rect_two.height);

    // 计算交集和并集的面积
    int intersection_area = std::max(0, x2 - x1) * std::max(0, y2 - y1);
    int box1_area = rect_one.width * rect_one.height;
    int rect_two_area = rect_two.width * rect_two.height;
    int union_area = box1_area + rect_two_area - intersection_area;

    // 计算IOU值
    float iou = static_cast<float>(intersection_area) / union_area;
    return iou;
}

/// @brief 用当前帧识别到的待打机框和上一帧的备选框进行iou操作，从而确定当前帧的待打击框，
///        如果上一帧没有待打击框，计算各个框到R框的距离,根据内圆覆盖流水灯和能量机关的特
///        殊性，离着R框最近的框必定是待打击框,在选取当前帧的打击框后，将选定框绕R框旋转，
///        从而生成当前帧的备选框。
/// @param image 经过hsv二值化处理和内外圆分割处理的图像
void Match::MatchBoarding(const cv::Mat &image) {
    std::vector<std::vector<cv::Point>> contours; // 存放临时轮廓的vector向量
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, 0, 2);
    for (auto &contour : contours) {
        cv::RotatedRect rect = cv::minAreaRect(contour);
        BoardBox box = BoardBox(rect);
        now_image.pre_box.push_back(box);
    }
    if (!last_image.rel_bbox.empty()) {
        for (auto &boardbox : now_image.pre_box) {
            for (auto &last_rel_box : last_image.pre_ch_box) {
                float iou = ComputeIou(boardbox, last_rel_box);
                if (iou > IOU_THRESOLD) {
                    now_image.rel_bbox.push_back(boardbox);
                }
            }
        }
        // 创建备选框
        if (!now_image.rel_bbox.empty()) {
            now_image.pre_ch_box.push_back(now_image.rel_bbox[0]);
            for (int i = 1; i < 6; i++) {
                 now_image.pre_ch_box.push_back(GetPrepareBox(now_image.rel_box.center, 72 * i, now_image.pre_ch_box[0]));
                 std::cout << "matched box" << std::endl;
            }
        } else {
            std::cout << "not matched box" << std::endl;
        }
    } else {
        std::vector<float> distances;
        for (int i = 0; i < now_image.pre_box.size(); i++) {
            float distance = std::sqrt(abs(now_image.rel_box.center.x - now_image.pre_box[i].center.x) * abs(now_image.rel_box.center.x - now_image.pre_box[i].center.x) + abs(now_image.rel_box.center.y - now_image.pre_box[i].center.y) * abs(now_image.rel_box.center.y - now_image.pre_box[i].center.y));
            distances.push_back(distance);
        }
        auto min = std::min_element(distances.begin(), distances.end());
        if (min != distances.end()) {
            int index = std::distance(distances.begin(), min);
            now_image.rel_bbox.push_back(now_image.pre_box[index]);
             now_image.pre_ch_box.push_back(now_image.rel_bbox[0]);
            for (int i = 1; i < 6; i++) {  
                 now_image.pre_ch_box.push_back(GetPrepareBox(now_image.rel_box.center, 72 * i, now_image.rel_bbox[0]));
            }
        }
        distances.clear();
        
    }
    last_image = now_image;    
} 
/// @brief 用于生成备选框
/// @param roate_center 旋转中心 
/// @param rotated_angle  旋转角度
/// @param rect 用于生成备选框的矩形
/// @return 生成的备选框
BoardBox Match::GetPrepareBox(const cv::Point2f &roate_center, const float &rotated_angle, const BoardBox &rect) {
    cv::Mat rot = cv::getRotationMatrix2D(roate_center, rotated_angle, 1.0);
    cv::Point2f point[4];
    rect.rect.points(point);
    std::vector<cv::Point2f> vertices;
    for (int i = 0; i < 4; ++i) {
        point[i] = cv::Point2f(rot.at<double>(0, 0) * point[i].x + rot.at<double>(0, 1) * point[i].y + rot.at<double>(0, 2),
                                  rot.at<double>(1, 0) * point[i].x + rot.at<double>(1, 1) * point[i].y + rot.at<double>(1, 2));
        vertices.push_back(point[i]);
    }
    cv::RotatedRect new_rect = cv::minAreaRect(vertices);
    BoardBox new_box(new_rect);
    vertices.clear();
    return new_box;
}

/// @brief 用于识别能量机关的集成函数
/// @param image 原图像
/// @param type 是否绘制备选框
void Match::Run(const cv::Mat &image, std::string type) {
    // AdjustNumber my_adjust(allinformations);
    // allinformations = my_adjust.AdjustDynamic();
    ImageTrackle my_trackle;
    cv::Mat result = my_trackle.ImageTrackleHSV(image);
    cv::imshow("二值化图", result);  
    MatchRBox(result);
    cv::Mat resulut_cricle = my_trackle.ImageTrackleCricle(result, now_image.rel_box);
    MatchBoarding(resulut_cricle);
    cv::Point2f points[4];
    now_image.rel_box.box_rect.points(points);
    for (int i = 0; i < 4; i++) {   
        cv::line(image, points[i], points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
    }
    cv::Point2f points_bbox[4];
    //绘制正确框
    for (auto &box : now_image.rel_bbox) {
        box.rect.points(points_bbox);
        for (int i = 0; i < 4; i++) {   
            cv::line(image, points_bbox[i], points_bbox[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }
    if (type == "Yes") {
        cv::Point2f points_pre_box[4];
        for (auto &box : last_image.pre_ch_box) {
            box.rect.points(points_pre_box);
            for (int i = 0; i < 4; i++) {   
                cv::line(image, points_pre_box[i], points_pre_box[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
            }
        }
    }
    cv::imshow("二值化图被圆形处理", resulut_cricle);  
    cv::imshow("原图", image);
    cv::waitKey(1);
    now_image = ImageInformation(); // 清除当前帧
}