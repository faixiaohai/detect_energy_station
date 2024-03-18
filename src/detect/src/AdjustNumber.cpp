#include <detect.h>
using namespace cv;
AdjustNumber::AdjustNumber(const AllInformation &information) {
    informations = information;
    h_l = 0;
    h_up = 0;
    s_l = 0;
    s_up = 0;
    v_l = 0;
    v_up = 0;
    informations.in_r = 0;
    informations.out_r = 0;
}
cv::Mat ChangeCricle(const cv::Mat &image);
cv::Mat DrawPreImg(const cv::Mat &image);
static int h_l, s_l, v_l, h_up, s_up, v_up, in_r_start, out_r_start;
AllInformation allinformation;
static void ChangeHl(int number, void *userdata) {
    h_l = number;
    allinformation.lower_thresold = Scalar(number, s_l, v_l);
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << &image << std::endl;
    std::cout << "adjust h_l" << std::endl;
    DrawPreImg(*image);
}
void ChangeSl(int number, void *userdata) {
    s_l = number;
    allinformation.lower_thresold = Scalar(h_l, number, v_l);
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << "adjust h_s" << std::endl;
    DrawPreImg(*image);
}
void ChangeVl(int number, void *userdata) {
    v_l = number;
    allinformation.lower_thresold = Scalar(h_l, s_l, number);
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeHup(int number, void *userdata) {
    h_up = number;
    allinformation.up_thresold = Scalar(number, s_up, v_up);
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeSup(int number, void *userdata) {
    s_up = number;
    allinformation.up_thresold = Scalar(h_up, number, v_up);
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeVup(int number, void *userdata) {
    v_up = number;
    allinformation.up_thresold = Scalar(h_up, s_up, number);
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    DrawPreImg(*image);
}
void ChangeInr(int number, void *userdata) {
    in_r_start = number;
    allinformation.in_r = number;
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << "adjust_outr" << std::endl;
    ChangeCricle(*image);
}
void ChangeOutr(int number, void *userdata) {
    out_r_start = number;
    allinformation.out_r = number;
    cv::Mat* image = static_cast<cv::Mat*>(userdata);
    std::cout << "adjust_inr" << std::endl;
    ChangeCricle(*image);
}
void AdjustNumber::AdjustDynamic(cv::Mat &image) {
    std::cout << &image << std::endl;
    cv::namedWindow("self_defined", WINDOW_AUTOSIZE);
    cv::createTrackbar("h_l", "self_defined", &h_l, 255, ChangeHl,&image);
    cv::createTrackbar("s_l", "self_defined", &s_l, 255, ChangeSl,&image );
    cv::createTrackbar("v_l", "self_defined", &v_l, 255, ChangeVl,&image);
    cv::createTrackbar("h_up", "self_defined", &h_up, 255,ChangeHup,&image);
    cv::createTrackbar("s_up", "self_defined", &s_up, 255, ChangeSup,&image);
    cv::createTrackbar("v_up", "self_defined", &v_up, 255, ChangeVup,&image);
    cv::createTrackbar("in_r", "self_defined", &informations.in_r, 250, ChangeInr,&image);
    cv::createTrackbar("out_r", "self_defined", &informations.out_r, 500,ChangeOutr,&image);
}
cv::Mat DrawPreImg(const cv::Mat &image) {
    cv::Mat result_img;
    ImageTrackle my_trackle(allinformation);
    Match my_match(allinformation);
    result_img = my_trackle.ImageTrackleHSV(image);
    cv::imshow("预调1", result_img);
    cv::waitKey(100);
    return result_img;
}
cv::Mat ChangeCricle(const cv::Mat &image) {
    cv::Mat result_img;
    allinformation.max_area_rbox = 2000;
    allinformation.iou_thresold = 0.4;
    allinformation.in_r = in_r_start;
    allinformation.out_r = out_r_start;
    ImageTrackle my_trackle(allinformation);
    Match my_match(allinformation);
    result_img = my_trackle.ImageTrackleHSV(image);
    my_match.MatchRBox(result_img);
    std::cout << "rbox" << my_match.now_image.rel_box.center << std::endl;
    result_img = my_trackle.ImageTrackleCricle(result_img, my_match.now_image.rel_box);
    imshow("预调_cricle", result_img);
    cv::waitKey(100);
    return result_img;
}
AllInformation AdjustNumber::ReInformation() {
    informations.lower_thresold = cv::Scalar(h_l, s_l, v_l);
    informations.up_thresold = cv::Scalar(h_up, s_up, v_up);
    return informations;
}