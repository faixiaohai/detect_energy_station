/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 11:07:18
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-04-01 11:09:09
 * @FilePath: /eigen_other/src/detect/src/Match.cpp
 * @Description: 用于待打击框选取的一系列函数
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include <detect.h>
Match::Match() {}
/// @brief 对输入的图像进行寻找轮廓，框选轮廓，和上一帧的R框进行iou操作从而选定当前帧的R框，如果上一帧并没有R框，则计算所有轮廓到图像中心的距离，距离最短的为R框;
/// @param image 经过hsv二值化处理的图像
void Match::MatchRBox(const cv::Mat &image)
{
    std::vector<std::vector<cv::Point>> contours; // 存放临时轮廓的vector向量
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, 0, 2);
    now_image.contours = contours;
    cv::RotatedRect rotated_rect;
    for (auto &contour : contours)
    {
        if (cv::contourArea(contour) > RBOX_MAX_AREAM)
        {
            continue;
        }
        if (cv::contourArea(contour) < RBOX_MIN_AREA)
        {
            continue;
        }
        rotated_rect = cv::minAreaRect(contour);
        RBox rbox(rotated_rect);
        now_image.pre_rbox.push_back(rbox);
    }
    std::vector<float> iou_arry;
    if (last_image.rel_rbox.center.x != 0)
    {
        std::vector<std::pair<float, RBox>> data_rbox;
        for (int i = 0; i < now_image.pre_rbox.size(); i++)
        {
            float iou = Kit::CIou(last_image.rel_rbox.box_rect, now_image.pre_rbox[i].box_rect);
            data_rbox.push_back(std::make_pair(iou, now_image.pre_rbox[i]));
        }
        for (int i = 0; i < now_image.pre_rbox.size(); i++)
        {
            for (int j = i + 1; j < now_image.pre_rbox.size(); j++)
            {
                if (data_rbox[i].first > data_rbox[j].first)
                {
                    std::pair<float, RBox> temp = data_rbox[i];
                    data_rbox[i] = data_rbox[j];
                    data_rbox[j] = temp;
                }
            }
        }
        if (data_rbox.back().first > 0.2) {
            now_image.rel_rbox = data_rbox.back().second;
        } else {
            now_image.rel_rbox = last_image.rel_rbox;
        }
        std::cout << "iou = " << data_rbox.back().first << std::endl;
    }
    else
    {
        cv::Point2f image_center;
        image_center.x = image.size().width / 2;
        image_center.y = image.size().height / 2;
        std::vector<float> distances;
        for (int i = 0; i < now_image.pre_rbox.size(); i++)
        {
            float distance = std::sqrt(abs(image_center.x - now_image.pre_rbox[i].center.x) * abs(image_center.x - now_image.pre_rbox[i].center.x) + abs(image_center.y - now_image.pre_rbox[i].center.y) * abs(image_center.y - now_image.pre_rbox[i].center.y));
            distances.push_back(distance);
        }
        auto min = std::min_element(distances.begin(), distances.end());
        if (min != distances.end())
        {
            int index = std::distance(distances.begin(), min);
            if (distances[index] < RBOX_SUB_CENTER)
            {
                std::cout << "index" << index << std::endl;
                now_image.rel_rbox = now_image.pre_rbox[index];
                std::cout << "founed rbox in now image" << std::endl;
            }
            else
            {
                std::cout << "please move center" << std::endl;
            }
        }
        iou_arry.clear();
    }
}
/// @brief 对输入的图像进行寻找轮廓，框选轮廓，和上一帧的R框进行iou操作从而选定当前帧的R框，如果上一帧并没有R框，则计算所有轮廓到图像中心的距离，距离最短的为R框;
/// @param image 经过hsv二值化处理的图像

void Match::MatchRBoxReL(const cv::Mat &image)
{
    std::vector<std::vector<cv::Point>> contours; // 存放临时轮廓的vector向量
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, 0, 2);
    now_image.contours = contours;
    cv::RotatedRect rotated_rect;
    for (auto &contour : contours)
    {
        if (cv::contourArea(contour) > RBOX_MAX_AREAM)
        {
            continue;
        }
        if (cv::contourArea(contour) < RBOX_MIN_AREA)
        {
            continue;
        }
        rotated_rect = cv::minAreaRect(contour);
        RBox rbox(rotated_rect);
        now_image.pre_rbox.push_back(rbox);
    }
    std::vector<float> iou_arry;
    if (last_image.rel_rbox.center.x != 0)
    {
        std::vector<std::pair<float, RBox>> data_rbox;
        for (int i = 0; i < now_image.pre_rbox.size(); i++)
        {
            float iou = Kit::CIou(last_image.rel_rbox.box_rect, now_image.pre_rbox[i].box_rect);
            data_rbox.push_back(std::make_pair(iou, now_image.pre_rbox[i]));
        }
        for (int i = 0; i < now_image.pre_rbox.size(); i++)
        {
            for (int j = i + 1; j < now_image.pre_rbox.size(); j++)
            {
                if (data_rbox[i].first > data_rbox[j].first)
                {
                    std::pair<float, RBox> temp = data_rbox[i];
                    data_rbox[i] = data_rbox[j];
                    data_rbox[j] = temp;
                }
            }
        }
        if (data_rbox.back().first > 0.4) {
            // std::cout << "iou = " << data_rbox.back().first << std::endl;
            // now_image.rel_rbox = data_rbox.back().second;
            now_image.rel_rbox = last_image.rel_rbox;
        } else {
            now_image.rel_rbox = last_image.rel_rbox;
        }
        
    }
    
}
/// @brief 用当前帧识别到的待打机框和上一帧的备选框进行iou操作，从而确定当前帧的待打击框，
///        如果上一帧没有待打击框，计算各个框到R 框的距离,根据内圆覆盖流水灯和能量机关的特
///        殊性，离着R框最近的框必定是待打击框,在选取当前帧的打击框后，将选定框绕R框旋转，
///        从而生成当前帧的备选框。
/// @param image 经过hsv二值化处理和内外圆分割处理的图像
void Match::MatchBoarding(const cv::Mat &image)
{
    std::vector<std::vector<cv::Point>> contours; // 存放临时轮廓的vector向量
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, 0, 2);
    for (auto &contour : contours)
    {
        if (cv::contourArea(contour) < AdjustNumber::barinformation.min_bbox_area)
        {
            continue;
        }
        if (cv::contourArea(contour) > AdjustNumber::barinformation.max_bbox_area)
        {
            continue;
        }
        cv::RotatedRect rect = cv::minAreaRect(contour);
        now_image.pre_bbox.push_back(BoardBox(rect, 7, SLOT, contour));
    }
    if (!last_image.rel_bbox.empty())
    {   
        // if (image_type != MOTIVED) {
            for (auto &boardbox : now_image.pre_bbox)
            {
                for (auto &last_rel_prebox : last_image.pre_ch_box)
                {

                    float iou = Kit::ComputeIou(boardbox.rect.boundingRect(), last_rel_prebox.rect.boundingRect());
                    float disantce = Kit::ComputeDistance(now_image.rel_rbox.center, boardbox.rect.center);
                    if (iou > IOU_THRESOLD && disantce > AdjustNumber::barinformation.in_r && disantce < AdjustNumber::barinformation.out_r)
                    {
                        boardbox.id = last_rel_prebox.id;
                        now_image.rel_bbox.push_back(boardbox);
                    }
                }
            }
        // }
            //  else if (image_type == MOTIVED) {
        //     for (auto &box : now_image.pre_bbox) {
        //         float distance = Kit::ComputeDistance(box.rect.center, now_image.rel_rbox.box_rect.center);
        //         if (distance > AdjustNumber::barinformation.in_r - 50 && distance < AdjustNumber::barinformation.out_r) {
        //             now_image.rel_bbox.push_back(box);
        //         }
        //     }
        // }
        std::vector<BoardBox> and_bbox;
        for (int i = 0; i < 5; i++) {
            for (auto &box : now_image.rel_bbox) {
                if (box.id == i) {
                    and_bbox.push_back(box);
                }
            }
            if (and_bbox.size() == 2)
            {
                int id = and_bbox[0].id;
                std::cout << "合并两个装甲板" << std::endl;
                BoardBox box_one = and_bbox[0];
                BoardBox box_two = and_bbox[1];
                cv::Point2f vec_point_one[4];
                cv::Point2f vec_point_two[4];
                box_one.rect.points(vec_point_one);
                box_two.rect.points(vec_point_two);
                std::vector<std::pair<float, cv::Point2f>> data;
                for (cv::Point2f &point1 : vec_point_one)
                {
                    data.push_back(std::make_pair(Kit::ComputeDistance(point1, now_image.rel_rbox.center), point1));
                }
                for (cv::Point2f &point : vec_point_two)
                {
                    data.push_back(std::make_pair(Kit::ComputeDistance(point, now_image.rel_rbox.center), point));
                }
                for (int i = 0; i < 8; i++)
                {
                    for (int j = i + 1; j < 8; j++)
                    {
                        if (data[i].first > data[j].first)
                        {
                            std::pair<float, cv::Point2f> temp = data[i];
                            data[i] = data[j];
                            data[j] = temp;
                        }
                    }
                }
                std::vector<cv::Point> rel_point;
                rel_point.push_back(data[0].second);
                rel_point.push_back(data[1].second);
                rel_point.push_back(data[6].second);
                rel_point.push_back(data[7].second);
                std::cout << "被选中的点" << std::endl;
                for (auto &point : rel_point)
                {
                    std::cout << point << std::endl;
                }
                cv::RotatedRect brect = cv::minAreaRect(rel_point);
                cv::Point2f draw_point[4];
                brect.points(draw_point);
                for (int i = 0; i < 4; i++)
                {
                    cv::line(image, draw_point[i], draw_point[(i + 1) % 4], cv::Scalar(255, 0, 0), 8);
                }
                cv::imshow("原图1", image);
                std::vector<BoardBox> temp;
                for (auto &box_rel : now_image.rel_bbox)
                {
                    if (box_rel.id != box_one.id)
                    {
                        temp.push_back(box_rel);
                    }
                }
                BoardBox and_box = BoardBox(brect, box_one.id, SLOT, rel_point);
                temp.push_back(and_box);
                now_image.rel_bbox.clear();
                for (auto &boxo : temp)
                {
                    now_image.rel_bbox.push_back(boxo);
                }
                std::cout << "当前帧图像的正确大符数量" << now_image.rel_bbox.size() << std::endl;
                and_bbox.clear();
            }
            else if (and_bbox.size() == 3)
            {
                std::cout << "合并三个装甲板" << std::endl;
                std::vector<std::pair<float, BoardBox>> data_box;
                for (auto &point_center : and_bbox)
                {
                    data_box.push_back(std::make_pair(Kit::ComputeDistance(point_center.rect.center, now_image.rel_rbox.center), point_center));
                }
                for (int i = 0; i < 3; i++)
                {
                    for (int j = i + 1; j < 3; j++)
                    {
                        if (data_box[i].first > data_box[j].first)
                        {
                            std::pair<float, BoardBox> temp = data_box[i];
                            data_box[i] = data_box[j];
                            data_box[j] = temp;
                        }
                    }
                }
                BoardBox box_one = data_box[0].second;
                BoardBox box_two = data_box[2].second;
                cv::Point2f vec_point_one[4];
                cv::Point2f vec_point_two[4];
                box_one.rect.points(vec_point_one);
                box_two.rect.points(vec_point_two);
                std::vector<std::pair<float, cv::Point2f>> data;
                for (cv::Point2f &point1 : vec_point_one)
                {
                    data.push_back(std::make_pair(Kit::ComputeDistance(point1, now_image.rel_rbox.center), point1));
                }
                for (cv::Point2f &point : vec_point_two)
                {
                    data.push_back(std::make_pair(Kit::ComputeDistance(point, now_image.rel_rbox.center), point));
                }
                std::cout << "检测到需要合并的装甲板" << std::endl;
                for (int i = 0; i < 8; i++)
                {
                    for (int j = i + 1; j < 8; j++)
                    {
                        if (data[i].first > data[j].first)
                        {
                            std::pair<float, cv::Point2f> temp = data[i];
                            data[i] = data[j];
                            data[j] = temp;
                        }
                    }
                }
                std::vector<cv::Point> rel_point;
                rel_point.push_back(data[0].second);
                rel_point.push_back(data[1].second);
                rel_point.push_back(data[6].second);
                rel_point.push_back(data[7].second);
                std::cout << "被选中的点" << std::endl;
                // for (int i = 0; i < 4; i++) {
                //     cv::line(image, rel_point[i], rel_point[(i + 1) % 4], cv::Scalar(255, 0,0), 8);
                // }
                for (auto &point : rel_point)
                {
                    std::cout << point << std::endl;
                }
                cv::RotatedRect brect = cv::minAreaRect(rel_point);
                cv::Point2f draw_point[4];
                brect.points(draw_point);
                for (int i = 0; i < 4; i++)
                {
                    cv::line(image, draw_point[i], draw_point[(i + 1) % 4], cv::Scalar(255, 0, 0), 8);
                }
                cv::imshow("原图1", image);
                std::vector<BoardBox> temp;
                for (auto &box_rel : now_image.rel_bbox)
                {
                    if (box_rel.id != box_one.id)
                    {
                        temp.push_back(box_rel);
                    }
                }
                BoardBox and_box = BoardBox(brect, box_one.id, SLOT, rel_point);
                temp.push_back(and_box);
                now_image.rel_bbox.clear();
                for (auto &boxo : temp)
                {
                    now_image.rel_bbox.push_back(boxo);
                }
                std::cout << "当前帧图像的正确大符数量" << now_image.rel_bbox.size() << std::endl;
                and_bbox.clear();
            }
            and_bbox.clear();
        }
        if (image_type == MOTIVED) {
            for (auto &box : now_image.rel_bbox) {
                now_image.sloted_box.push_back(box);
            }
            // if (now_image.pre_ch_box.empty()) {
            //     for (auto &box : now_image.sloted_box) {
            //         now_image.pre_ch_box.push_back(box);
            //         now_image.pre_ch_box[0].id = 0;
            //         for (int i = 1; i < 5; i++)
            //         {
            //             cv::Rect rect = cv::boundingRect(box.contours);
            //             now_image.pre_ch_box.push_back(GetPrepareBox(now_image.rel_rbox.center, 72 * i, now_image.pre_ch_box[0]));
            //             // std::cout << "matched box" << std::endl;
            //         }
            //         break;
            //     }
            // }
            // return;
        }
        if (now_image.rel_bbox.size() == 1 && last_image.rel_bbox.size() < 4)
        {
            now_image.wait_slot_box.push_back(now_image.rel_bbox[0]);
        }
        if (now_image.rel_bbox.size() > last_image.rel_bbox.size())
        {
            std::vector<int> id_arry;
            for (int i = 0; i < now_image.rel_bbox.size(); i++)
            {
                id_arry.push_back(now_image.rel_bbox[i].id);
            }
            for (auto &bbox : last_image.rel_bbox)
            {
                for (auto &box : now_image.pre_bbox)
                {
                    if (box.id == bbox.id)
                    {
                        now_image.sloted_box.push_back(box);
                        int id = box.id;
                        auto it = std::find(id_arry.begin(), id_arry.end(), id);
                        if (it != id_arry.end())
                        {
                            id_arry.erase(it);
                        }
                    }
                }
            }
            for (auto &bbox : now_image.rel_bbox)
            {
                if (bbox.id == id_arry[0])
                {
                    now_image.wait_slot_box.push_back(bbox);
                }
            }
        }
        else if (now_image.rel_bbox.size() == last_image.rel_bbox.size())
        {
            for (auto &box : now_image.rel_bbox)
            {
                if (box.id == last_image.wait_slot_box[0].id)
                {
                    now_image.wait_slot_box.push_back(box);
                }
                else
                {
                    now_image.sloted_box.push_back(box);
                }
            }
        }
        // else if (now_image.rel_bbox.size() < last_image.rel_bbox.size())
        // {   
        //    if (last_image.rel_bbox.size() - now_image.rel_bbox.size() > 5) {
        //     now_image = last_image;
        //    }
        // }
        else if (now_image.rel_bbox.size() < last_image.rel_bbox.size() && last_image.rel_bbox.size() == 5 && last_image.sloted_box.size() == 4) {
            std::cout << "激活完毕" << std::endl;
            for (auto &box : last_image.rel_bbox) {
                last_image.sloted_box.push_back(last_image.wait_slot_box[0]);
                last_image.wait_slot_box.clear();
                image_type = MOTIVED;
            }
            return;
        }
        // 创建备选框
        if (now_image.pre_ch_box.empty())
        {
            if (now_image.sloted_box.empty())
            {
                for (auto box : now_image.rel_bbox)
                {
                    if (box.id == 0)
                    {
                        now_image.pre_ch_box.push_back(box);
                        now_image.pre_ch_box[0].id = 0;
                        for (int i = 1; i < 5; i++)
                        {
                            cv::Rect rect = cv::boundingRect(box.contours);
                            now_image.pre_ch_box.push_back(GetPrepareBox(now_image.rel_rbox.center, 72 * i, now_image.pre_ch_box[0]));
                            // std::cout << "matched box" << std::endl;
                        }
                    }
                }
            }
            else
            {
                for (auto box : now_image.sloted_box)
                {
                    if (box.id == 0)
                    {
                        now_image.pre_ch_box.push_back(box);
                        now_image.pre_ch_box[0].id = 0;
                        for (int i = 1; i < 5; i++)
                        {
                            cv::Rect rect = cv::boundingRect(box.contours);
                            now_image.pre_ch_box.push_back(GetPrepareBox(now_image.rel_rbox.center, 72 * i, now_image.pre_ch_box[0]));
                            // std::cout << "matched box" << std::endl;
                        }
                    }
                }
            }
        }
        else
        {
            std::cout << "not matched box" << std::endl;
        }
    }
    else
    {
        std::vector<float> distances;
        if (now_image.pre_bbox.size() != 0)
        {
            for (int i = 0; i < now_image.pre_bbox.size(); i++)
            {
                float distance = std::sqrt(abs(now_image.rel_rbox.center.x - now_image.pre_bbox[i].center.x) * abs(now_image.rel_rbox.center.x - now_image.pre_bbox[i].center.x) + abs(now_image.rel_rbox.center.y - now_image.pre_bbox[i].center.y) * abs(now_image.rel_rbox.center.y - now_image.pre_bbox[i].center.y));
                distances.push_back(distance);
                if (distance > AdjustNumber::barinformation.in_r && distance < AdjustNumber::barinformation.out_r)
                {
                    now_image.rel_bbox.push_back(now_image.pre_bbox[i]);
                }
            }
            now_image.rel_bbox[0].id = 0;
            now_image.wait_slot_box.push_back(now_image.rel_bbox[0]);
            now_image.pre_ch_box.push_back(now_image.rel_bbox[0]);
            for (int i = 1; i < 5; i++)
            {
                now_image.pre_ch_box.push_back(GetPrepareBox(now_image.rel_rbox.center, 72 * i, now_image.rel_bbox[0]));
            }
            distances.clear();
        }
    }
    last_image = now_image;
}
/// @brief 用于生成备选框
/// @param roate_center 旋转中心
/// @param rotated_angle  旋转角度
/// @param rect 用于生成备选框的矩形
/// @return 生成的备选框cosnt cv::Point2f &pt1, const cv::Point2f &pt2
BoardBox Match::GetPrepareBox(const cv::Point2f &roate_center, const float &rotated_angle, const BoardBox &rect)
{
    cv::Mat rot = cv::getRotationMatrix2D(roate_center, rotated_angle, 1.0);
    // cv::Rect rect_max = cv::boundingRect(rect.contours);
    cv::Point2f point[4];
    rect.rect.points(point);
    std::vector<cv::Point> vertices;
    for (int i = 0; i < 4; ++i)
    {
        point[i] = cv::Point(rot.at<double>(0, 0) * point[i].x + rot.at<double>(0, 1) * point[i].y + rot.at<double>(0, 2),
                             rot.at<double>(1, 0) * point[i].x + rot.at<double>(1, 1) * point[i].y + rot.at<double>(1, 2));
        vertices.push_back(point[i]);
    }
    cv::RotatedRect new_rect = cv::minAreaRect(vertices);
    BoardBox new_box(new_rect, rotated_angle / 72, NOLIGHT, vertices);
    vertices.clear();
    return new_box;
}

/// @brief 用于识别能量机关的集成函数
/// @param image 原图像
/// @param type 是否绘制备选框
void Match::Run(const cv::Mat &image, std::string type, std::string type_other)
{
    ImageTrackle my_trackle;
    cv::Mat result = my_trackle.ImageTrackleHSV(image);
    cv::imshow("find r", result);
    MatchRBoxReL(result);
    cv::Mat result_cricle;
    if (now_image.rel_rbox.center.x != 0)
    {
        result_cricle = my_trackle.ImageTrackleCricle(result, now_image.rel_rbox);
        MatchBoarding(result_cricle);
    }

    cv::Point2f points[4];
    now_image.rel_rbox.box_rect.points(points);
    for (int i = 0; i < 4; i++)
    {
        cv::line(image, points[i], points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
    }
    cv::Point2f points_bbox[4];
    // 绘制待打击框
    for (auto &box : now_image.wait_slot_box)
    {
        // cv::Rect rect = cv::boundingRect(box.contours);
        // cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 2);
        std::vector<cv::Point2f> Points;
        for (auto &box : now_image.wait_slot_box)
        {
            box.rect.points(Points);
            for (int i = 0; i < 4; i++)
            {
                cv::line(image, Points[i], Points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            }
        }
    }
    if (type_other == "draw_sloted")
    {
        // for (auto &box : now_image.sloted_box) {
        //     cv::Rect rect = cv::boundingRect(box.contours);
        //     cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 2);
        // }
        std::vector<cv::Point2f> Points;
        for (auto &box : now_image.sloted_box)
        {
            box.rect.points(Points);
            for (int i = 0; i < 4; i++)
            {
                cv::line(image, Points[i], Points[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
            }
        }
        if (type == "draw_pre")
        {
            cv::Point2f points_pre_box[4];
            for (auto &box : last_image.pre_ch_box)
            {
                box.rect.points(points_pre_box);
                // for (int i = 0; i < 4; i++) {
                //     cv::line(image, points_pre_box[i], points_pre_box[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
                //     cv::String id = std::to_string(box.id);
                //     cv::putText(image, id, box.center, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
                // }
                cv::Rect rect = box.rect.boundingRect();
                cv::rectangle(image, rect, cv::Scalar(0, 0, 255), 2);
            }
        }
        cv::imshow("二值化图被圆形处理", result_cricle);
        cv::imshow("原图", image);
        cv::waitKey(0);
        now_image = ImageInformation(); // 清除当前帧
    }
}