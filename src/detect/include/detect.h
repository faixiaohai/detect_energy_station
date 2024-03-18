#include <iostream>
#include <opencv2/opencv.hpp>

struct AllInformation
{
    /* data */
    cv::Scalar up_thresold;
    cv::Scalar lower_thresold;
    int max_area_rbox;
    int in_r;
    int out_r;
    int iou_thresold;
};

enum DetectColor {
    BLUE,
    RED
};
class AdjustNumber {
public:
    AdjustNumber(const AllInformation &information);
    void AdjustDynamic(cv::Mat &image);
    AllInformation ReInformation();
    // void ChangeHl(int number, void *);
    // cv::Mat DrawPreImg(const cv::Mat &image);
public:
    AllInformation informations;
    int h_l;
    int s_l;
    int v_l;
    int h_up;
    int s_up;
    int v_up;
};

class RBox {
public:
    RBox();
    RBox(const cv::RotatedRect &rect);
public:
    cv::RotatedRect box_rect;
    cv::Point2f center;
    float height;
    float width;
};
class BoardBox {
public:
    BoardBox();
    BoardBox(const cv::RotatedRect &rrect);
public:
    cv::RotatedRect rect;
    cv::Point2f center;
    float height;
    float width;
};

class ImageTrackle {
public:
    ImageTrackle(const AllInformation &information);
    cv::Mat ImageTrackleCricle(const cv::Mat &image, const RBox &box);
    cv::Mat ImageTrackleHSV(const cv::Mat &image);   
public:
    AllInformation trackle_information;
    cv::Scalar up_thresold;
    cv::Scalar lower_thresold;
    float in_r;
    float out_r;
};
class ImageInformation {
public:
    ImageInformation();
public:
    std::vector<RBox> pre_rbox; 
    RBox rel_box;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<BoardBox> pre_box;
    std::vector<BoardBox> rel_bbox;
    std::vector<BoardBox> pre_ch_box;
};
class Match {
public:
    Match(const AllInformation &allinformation);
    void Run(const cv::Mat &image, int draw_pre_box);
    void MatchRBox(const cv::Mat &image);
    void MatchBoarding(const cv::Mat &image);
    float ComputeIou(const RBox &rect_one, const RBox &rect_two);
    float ComputeIou(const BoardBox &rect_one, const BoardBox &rect_two);
    BoardBox GetPrepareBox(const cv::Point2f &rotate_center, const float &rotated_angel, const BoardBox &rotated_rect);
public:
    AllInformation allinformations;
    ImageInformation now_image;
    ImageInformation last_image;
};

