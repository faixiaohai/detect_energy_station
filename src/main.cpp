/*
 * @Author: myq 2127800097@qq.com
 * @Date: 2024-03-16 11:05:45
 * @LastEditors: myq 2127800097@qq.com
 * @LastEditTime: 2024-03-31 11:29:57
 * @FilePath: /eigen_other/src/main.cpp
 * @Description:  
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include <iostream>
#include <detect.h>
int main()
{
    Match my_match;
    cv::VideoCapture my_cap(PATH);
    cv::Mat frame, img_pre;
    while (1) {
        my_cap.read(frame);
        char key = cv::waitKey(30);
        if (key == 27) {
            break;
        }
        cv::imshow("预选", frame);
        // cv::waitKey(0);
    }
    cv::destroyAllWindows();

    AdjustNumber::AdjustDynamic(frame); // 创建一个用于调节hsv上下阈值的bar。
    cv::waitKey(0); // 暂停界面用于调节阈值
    cv::RotatedRect rotatedrect = Kit::SelectRoatedRect(frame);
    RBox rbox = RBox(rotatedrect);
    my_match.now_image.rel_rbox = rbox;
    my_match.last_image.rel_rbox = rbox;
    cv::destroyAllWindows();
    while (1)
    {
        my_cap.read(frame);
        if (frame.empty())
        {
            std::cout << "读取失败" << std::endl;
            break;
        }
        my_match.Run(frame, "No", "draw_sloted"); // run集成运行函数 
    }
}