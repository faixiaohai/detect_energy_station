#include <detect.h>
BoardBox::BoardBox() {}
BoardBox::BoardBox(const cv::RotatedRect &rrect) {
    rect = rrect;
    center = rrect.center;
    height = rrect.size.height;
    width  = rrect.size.width;
}