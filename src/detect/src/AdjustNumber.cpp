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
// static int h_l, s_l, v_l, h_up, s_up, v_up, in_r_start, out_r_start;
// static Scalar hsv_lower = Scalar(h_l, s_l, v_l);
// static Scalar hsv_up = Scalar(h_up, s_up, v_up);
// static int in_r_;
// static int out_r_;
static AllInformation information_temp;
static AllInformation allinformation;
static void ChangeHl(int number, void *) {

    // allinformation.lower_thresold = Scalar(number, s_l, v_l);    
}
static void ChangeSl(int number, void *) {
    // allinformation.lower_thresold = Scalar(h_l, number, v_l);
}
static void ChangeVl(int number, void *) {
    // allinformation.lower_thresold = Scalar(h_l, s_l, number);
}
static void ChangeHup(int number, void *) {

    // allinformation.up_thresold = Scalar(number, s_up, v_up);
}
static void ChangeSup(int number, void *) {

    // allinformation.up_thresold = Scalar(h_up, number, v_up);
   
}
static void ChangeVup(int number, void *) {

    // allinformation.up_thresold = Scalar(h_up, s_up, number);
}
static void ChangeInr(int number, void *) {
    // std::cout << "in_r = " << number << std::endl;
}
static void ChangeOutr(int number, void *) {
    
    // std::cout << "out_r = " << number << std::endl;
}
void AdjustNumber::AdjustDynamic(const cv::Mat image) {
    cv::Mat out_img;
    cv::namedWindow("self_defined", WINDOW_AUTOSIZE);
    cv::createTrackbar("h_l", "self_defined", &h_l, 255, ChangeHl);
    cv::createTrackbar("s_l", "self_defined", &s_l, 255, ChangeSl);
    cv::createTrackbar("v_l", "self_defined", &v_l, 255, ChangeVl);
    cv::createTrackbar("h_up", "self_defined", &h_up, 255, ChangeHup);
    cv::createTrackbar("s_up", "self_defined", &s_up, 255, ChangeSup);
    cv::createTrackbar("v_up", "self_defined", &v_up, 255, ChangeVup);
    cv::createTrackbar("in_r", "self_defined", &informations.in_r, 250, ChangeInr);
    cv::createTrackbar("out_r", "self_defined", &informations.out_r, 500,ChangeOutr);
}
AllInformation AdjustNumber::ReInformation() {
    informations.lower_thresold = cv::Scalar(h_l, s_l, v_l);
    informations.up_thresold = cv::Scalar(h_up, s_up, v_up);
    return informations;
}