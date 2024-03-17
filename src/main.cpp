#include <iostream>
#include <detect.h>
#define PATH "../../file/1.mp4"
int main() {
    AllInformation allinformation;
    allinformation.max_area_rbox = 2000;
    allinformation.iou_thresold = 0.4;
    AdjustNumber my_adjust(allinformation);
    // allinformation.lower_thresold = cv::Scalar(0, 84, 254);
    // allinformation.up_thresold = cv::Scalar(60, 255, 255);
    // allinformation.in_r = 90;
    // allinformation.out_r = 180;
    cv::VideoCapture my_cap(PATH);
    cv::Mat frame;
    my_cap.read(frame);
    my_adjust.AdjustDynamic(frame);
    cv::waitKey(0);
    allinformation = my_adjust.ReInformation();
    std::cout << my_adjust.informations.in_r << std::endl;
    Match my_match(allinformation);
    while (1) {
        my_cap.read(frame); 
        if (frame.empty()) {
            std::cout << "读取失败" << std::endl;
            break;  
        }
        my_match.Run(frame);
    }

}