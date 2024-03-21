#include <iostream>
#include <opencv2/opencv.hpp>
#define PATH "../../file/1.mp4"
#define RBOX_MAX_AREAM  2000
#define IOU_THRESOLD  0.5
/// @brief  用bar动态调节的数值
struct BarInformation
{   
    int h_l;
    int s_l;
    int v_l;
    int h_up;
    int s_up;
    int v_up;
    int in_r;
    int out_r;
};

/// @brief 识别的颜色
enum DetectColor {
    BLUE,
    RED
};

/// @brief 生成bar界面
class AdjustNumber {
public:
    AdjustNumber();
    static void AdjustDynamic(cv::Mat &image);
public:
    static BarInformation barinformation;
};

/// @brief R框
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

/// @brief 待击打框
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

/// @brief 图像处理
class ImageTrackle {
public:
    ImageTrackle();
    cv::Mat ImageTrackleCricle(const cv::Mat &image, const RBox &box);
    cv::Mat ImageTrackleHSV(const cv::Mat &image);   
public:
    cv::Scalar up_thresold;
    cv::Scalar lower_thresold;
    int in_r;
    int out_r;
};

/// @brief 图像的一系列信息
class ImageInformation {
public:
    ImageInformation() {};
public:
    std::vector<RBox> pre_rbox; 
    RBox rel_box;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<BoardBox> pre_box;
    std::vector<BoardBox> rel_bbox;
    std::vector<BoardBox> pre_ch_box;
};

/// @brief 匹配待打击框
class Match {
public:
    Match();
    void Run(const cv::Mat &image, std::string type = "No");
    void MatchRBox(const cv::Mat &image);
    void MatchBoarding(const cv::Mat &image);
    float ComputeIou(const RBox &rect_one, const RBox &rect_two);
    float ComputeIou(const BoardBox &rect_one, const BoardBox &rect_two);
    BoardBox GetPrepareBox(const cv::Point2f &rotate_center, const float &rotated_angel, const BoardBox &rotated_rect);
public:
    ImageInformation now_image;  //当前帧
    ImageInformation last_image; // 上一帧
};

