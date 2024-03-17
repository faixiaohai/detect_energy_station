#include <detect.h>
RBox::RBox() {
    
}
RBox::RBox(const cv::RotatedRect &rect) {
    box_rect = rect;
    center = rect.center;
    height = rect.size.height;
    width = rect.size.width;

}