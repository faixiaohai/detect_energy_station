#ifndef CAMERA_START_H
#define CAMERA_START_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <map>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/video/video.hpp>

#include "MvCameraControl.h"
class Camera {
public:
    Camera(const int &index);
    void PrintDeviceInfo();
    void CloseCamera();

    void SetExposure(const int value);

    void SetGain(const int value);

    void* GetHandle() const { return m_handle; }
    bool readImageData(MV_FRAME_OUT& buff, const unsigned int& timeout_ms);


    std::string GetDeviceInfo();

    bool OpenCamera(const float &exposurenumber);
    void GetPic(cv::Mat*  srcimg);
private:
    int m_index;
    void* m_handle; // 相机句柄
    bool m_if_exit; //是否退出的标志
    int m_return_data; // 返回值
    unsigned int m_image_data_size; // 图像数据大小
    unsigned char *m_image_cache; // 图像数据缓存区
    MV_CC_DEVICE_INFO* m_device_info; // 设备信息指针
    MV_CC_DEVICE_INFO_LIST m_device_list; // 设备列表
    MVCC_INTVALUE m_param; // 参数;
    MV_FRAME_OUT m_out_frame; // 输出帧
    MV_CC_PIXEL_CONVERT_PARAM m_cvt_param; // 像素转换参数
};
#endif // CAMERA_START_H