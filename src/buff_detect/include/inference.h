#ifndef INFERENCE_H
#define INFERENCE_H

#include <iostream>
#include <fstream>
#include <ctime>



#include <openvino/openvino.hpp>
#include <inference_engine.hpp>


#include <eigen3/Eigen/Core>


#include <opencv2/opencv.hpp>

static constexpr int k_NumberClassification = 8;
static constexpr int k_NumberColor = 4;
static constexpr int k_InferenceResultPollMaxVal = 8;  // 最多保留的推理出的R数量(保留概率更大的)
static constexpr float k_BoundBoxConfidenceThreshold = 0.01f;  // 推理输出的边框的概率的阈值(en: bounding box confidence threshold)
static constexpr float k_NmsThreshold = 0.35f;  // 非极大值抑制-阈值(en: Non-Maximum Suppression)
static constexpr float k_MergeConfidenceError = 0.15f;  // 重合置信度误差
static constexpr float k_UseToMeanIouThreshold = 0.9f;  // 高于该阈值的iou可以被用来均值化R框的四个顶点以减少误差
// struct GridAndStride {
//     int grid_x;
//     int grid_y;
//     int stride;  // 步长
// };
struct GridAndStride {
    int grid_x;
    int grid_y;
    int stride;  // 步长
};

struct InferenceResult {
    int class_id;  // Blue: 0; Red: 1; 灰色: 2
    float confidence; // 置信度
    cv::Rect_<float> rect;  // R框包围矩形
    cv::Point2f r_apex[4];
    std::vector<cv::Point2f> points;
};

class OpenvinoInference {
public:
    static constexpr int INPUT_HEIGHT = 416;
    static constexpr int INPUT_WIDTH = 416;

    OpenvinoInference(const std::string &model_path, const std::string &inferplace);  

    void StartInfer(const cv::Mat &image, std::vector<InferenceResult> *result);

    void TrackleData(std::vector<InferenceResult> *infer_request);
    void ProcessYoloV5Output(const float *outputData, const float &confidenceThreshold,  const std::vector<GridAndStride> &grid_strides, std::vector<InferenceResult> &infer_request);

    void ProcessYoloV5Output(const float *outputData, const float &confidenceThreshold,  const float &image_width, const float &image_heihgt, std::vector<InferenceResult> &inter_request);

    

    InferenceEngine::Blob::Ptr WarpMatToBlob(const cv::Mat &image);

    void GenerateGridsAndStride(const int& target_w, const int& target_h, const std::vector<int>& strides,
                            std::vector<GridAndStride>* grid_strides);
// 后处理函数，解析并处理模型输出

    void nonMaximumSuppression(std::vector<InferenceResult> &selected_armors, std::vector<InferenceResult>& inference_armors, const float& threshold);

private:
    InferenceEngine::InputInfo::Ptr m_input_info; //输入的一系列数据
    std::string m_input_name; //输入数据的名字
    InferenceEngine::DataPtr m_output_info; //输出的一系列数据
    std::string m_output_name; // 输出数据的名字0
    InferenceEngine::InferRequest m_inferrequest; //推理器
    InferenceEngine::Blob::Ptr m_output_blob; // 输出的blob数据
    InferenceEngine::ExecutableNetwork m_executableNetwork; 
    ov::CompiledModel m_network_ov; 
    InferenceEngine::CNNNetwork m_network;
    const float* m_outputData;
    int m_src_width;
    Eigen::Matrix<float, 3, 3> m_transfrom_matrix;
    int m_src_height;
};

#endif // INFERENCE_H

