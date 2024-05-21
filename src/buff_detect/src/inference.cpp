#include <inference.h>
using namespace InferenceEngine;
OpenvinoInference::OpenvinoInference(const std::string &model_path, const std::string &inferplace)
{
    // 初始化Inference Engin
    Core ie;

    m_network = ie.ReadNetwork(model_path);

    // 设置输入配置
    m_input_info = m_network.getInputsInfo().begin()->second; // 获取输入数据的一系列数据
    m_input_name = m_network.getInputsInfo().begin()->first;  // 获取输入数据的名称
    m_input_info->setLayout(Layout::NCHW);                    // 将输入数据的图像数据改为 nhwc
    m_input_info->setPrecision(Precision::FP32);              // 方法将该输入数据的精度设置为无符号8位整数（U8）。这种数据类型通常用于图像数据，因为它能够有效地表示像素值（0-255范围内）。

    // 设置输出配置
    m_output_info = m_network.getOutputsInfo().begin()->second; // 获取输出的数据
    m_output_name = m_network.getOutputsInfo().begin()->first;  // 获取输出数据的名字
    m_output_info->setPrecision(Precision::FP32);

    // 加载模型到设备
    m_executableNetwork = ie.LoadNetwork(m_network, inferplace); // 在cpu加载神经网络模型，也可以用gpu等等
    InferenceEngine::InputsDataMap inputInfo(m_network.getInputsInfo());
    InferenceEngine::OutputsDataMap outputInfo(m_network.getOutputsInfo());
    std::cout << "Model Inputs:" << std::endl;
    for (auto &&input : inputInfo)
    {
        std::string input_name = input.first;
        SizeVector input_shape = input.second->getTensorDesc().getDims();
        std::cout << "Name: " << input_name << ", Shape: ";
        for (size_t i = 0; i < input_shape.size(); ++i)
        {
            std::cout << input_shape[i] << " ";
        }
        std::cout << std::endl;
    }

    // 打印模型输出信息
    std::cout << "\nModel Outputs:" << std::endl;
    for (auto &&output : outputInfo)
    {
        std::string output_name = output.first;
        SizeVector output_shape = output.second->getTensorDesc().getDims();
        std::cout << "Name: " << output_name << ", Shape: ";
        for (size_t i = 0; i < output_shape.size(); ++i)
        {
            std::cout << output_shape[i] << " ";
        }
        std::cout << std::endl;
    }
}

void OpenvinoInference::StartInfer(const cv::Mat &image, std::vector<InferenceResult> *result)
{
    cv::Mat out_image;
    image.convertTo(out_image, CV_32F);
    cv::Mat split[3];
    cv::split(out_image, split);
    m_inferrequest = m_executableNetwork.CreateInferRequest();
    int size = out_image.cols * out_image.rows;
    Blob::Ptr input_blob = m_inferrequest.GetBlob(m_input_name);
    float *input_data = input_blob->buffer().as<float *>();
    for (int i = 0; i < 3; i++)
    {
        std::memcpy(input_data + i * size, split[i].data, size * sizeof(float));
    }
    m_inferrequest.SetBlob(m_input_name, input_blob);
    m_inferrequest.Infer();
    m_output_blob = m_inferrequest.GetBlob(m_output_name);
    m_outputData = m_output_blob->buffer().as<const float *>();
    TrackleData(result);
}
void OpenvinoInference::GenerateGridsAndStride(const int &target_w, const int &target_h, const std::vector<int> &strides, std::vector<GridAndStride> *grid_strides)
{

    for (const auto &stride : strides)
    {
        for (int i = 0; i < target_w / stride; i++)
        {
            for (int j = 0; j < target_h / stride; j++)
            {
                GridAndStride grid_stride;
                grid_stride.grid_x = i * stride;
                grid_stride.grid_y = j * stride;
                grid_stride.stride = stride;
                grid_strides->push_back(grid_stride);
            }
        }
    }
}
void OpenvinoInference::ProcessYoloV5Output(const float *outputData, const float &confidenceThreshold,  const std::vector<GridAndStride> &grid_strides, std::vector<InferenceResult> &infer_request)
{
    
    for (int anchor_idx = 0; anchor_idx < grid_strides.size(); anchor_idx++)
    {
        // 通过网格、步长、偏移获取值
        const auto grid0 = static_cast<float>(grid_strides[anchor_idx].grid_x);
        const auto grid1 = static_cast<float>(grid_strides[anchor_idx].grid_y);
        const auto stride = static_cast<float>(grid_strides[anchor_idx].stride);
        const int basic_position = anchor_idx * (9 + k_NumberColor + k_NumberClassification);
        float x_1 = (outputData[basic_position + 0] + grid0) * stride;
        float y_1 = (outputData[basic_position + 1] + grid1) * stride;
        float x_2 = (outputData[basic_position + 2] + grid0) * stride;
        float y_2 = (outputData[basic_position + 3] + grid1) * stride;
        float x_3 = (outputData[basic_position + 4] + grid0) * stride;
        float y_3 = (outputData[basic_position + 5] + grid1) * stride;
        float x_4 = (outputData[basic_position + 6] + grid0) * stride;
        float y_4 = (outputData[basic_position + 7] + grid1) * stride;
        float box_probability = outputData[basic_position + 9];
        int class_id = outputData[basic_position + 8];
        if (box_probability >= 0.3) {
            InferenceResult r_box;
            Eigen::Matrix<float, 3, 4> r_apex_destination;  // 目标顶点
            r_apex_destination << x_1, x_2, x_3, x_4,
                                     y_1, y_2, y_3, y_4,
                                     1,   1,   1,   1;
            for (int i = 0; i < 4; i++)
            {
                r_box.r_apex[i] = cv::Point2f(r_apex_destination(0, i), r_apex_destination(1, i));
                r_box.points.push_back(r_box.r_apex[i]);
            }
            std::vector<cv::Point2f> tmp(r_box.r_apex, r_box.r_apex + 4);
            r_box.rect = cv::boundingRect(tmp);
            r_box.class_id = class_id;
            r_box.confidence = box_probability;
            infer_request.push_back(r_box);
        }
    }
}

void OpenvinoInference::ProcessYoloV5Output(const float *outputData, const float &confidenceThreshold,  const float &image_width, const float &image_heihgt, std::vector<InferenceResult> &inter_request) {

    for (int i = 0; i < 25200; ++i) {
        int offset = i * 6;  // 每个预测框有6个值：[x_min, y_min, x_max, y_max, confidence, class_id]
        float x_min = outputData[offset + 0];
        float y_min = outputData[offset + 1];
        float x_max = outputData[offset + 2];
        float y_max = outputData[offset + 3];
        float confidence = outputData[offset + 4];
        int class_id = static_cast<int>(outputData[offset + 5]);

        if (confidence >= confidenceThreshold) {
            // 创建检测结果对象并添加到结果列表
            InferenceResult result;
            result.points.push_back(cv::Point2f(x_min, y_min));
            result.points.push_back(cv::Point2f(x_max, y_max));
            result.confidence = confidence;
            result.class_id = class_id;
            // 将检测结果存储到结果列表中
            inter_request.push_back(result);
            std::cout << class_id << std::endl;
            std::cout << "point" << result.points[0].x  << " " << result.points[0].y << std::endl;
            std::cout << "point" << result.points[1].x << " " << result.points[1].y << std::endl;
        }
    }
}

void OpenvinoInference::TrackleData(std::vector<InferenceResult> *infer_result) {

    if (m_outputData == nullptr) return;
    std::vector<InferenceResult> tmp_r;
    std::vector<GridAndStride> grid_strides;
    std::vector<int> strides = { 8, 16, 32 };
    // 解析
    GenerateGridsAndStride(OpenvinoInference::INPUT_WIDTH, OpenvinoInference::INPUT_HEIGHT, strides, &grid_strides);
    ProcessYoloV5Output(m_outputData, k_BoundBoxConfidenceThreshold, 640, 640, tmp_r);
    
}


