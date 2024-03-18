#include <iostream>
#include <detect.h>
#define PATH "../../file/1.mp4"

int main() {
    // 不变参数，其余参数用bar调
    AllInformation allinformation;
    allinformation.max_area_rbox = 2000;
    allinformation.iou_thresold = 0.4;
    AdjustNumber my_adjust(allinformation);
    cv::VideoCapture my_cap(PATH);
    cv::Mat frame, img_pre;
    my_cap.read(frame);
    my_adjust.AdjustDynamic(frame); 
    cv::waitKey(0);
    allinformation = my_adjust.ReInformation();
    Match my_match(allinformation);
    while (1) {
        my_cap.read(frame); 
        if (frame.empty()) {
            std::cout << "读取失败" << std::endl;
            break;  
        }
        my_match.Run(frame, 0); // run集成运行函数 0不绘制备选库，1绘制
    }

}