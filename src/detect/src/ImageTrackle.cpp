#include <detect.h>
ImageTrackle::ImageTrackle(const cv::Scalar &lower, const cv::Scalar &up) {
    up_thresold = up;
    lower_thresold = lower;
}
cv::Mat ImageTrackle::ImageTrackleHSV(const cv::Mat &image) {
    cv::Mat output_image, hsv_image;
    cv::cvtColor(image,hsv_image, cv::COLOR_BGR2HSV);   
    cv::inRange(hsv_image, lower_thresold, up_thresold, output_image);
    cv::Mat kernel_b = cv::getStructuringElement(0,cv::Size(2, 2));
    cv::Mat kernel_a = cv::getStructuringElement(0,cv::Size(2, 2));
    cv::morphologyEx(output_image, output_image, 1, kernel_a, cv::Point(-1, -1), 3 );
    return output_image;
}
cv::Mat ImageTrackle::ImageTrackleCricle(const cv::Mat &image,
                                         const RBox &box,
                                         const float &in_r,
                                         const float &out_r) {
    cv::Mat out_img;
    cv::circle(image, box.center, in_r, cv::Scalar(0, 0, 0), -1);
    cv::circle(image, box.center, out_r, cv::Scalar(0, 0, 0), 1);
    cv::Mat kernel_a = cv::getStructuringElement(0,cv::Size(3, 3));
    cv::morphologyEx(image, image, 1, kernel_a, cv::Point(-1, -1), 10 );
    return image;
}