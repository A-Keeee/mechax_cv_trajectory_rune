#ifndef DETECT_HPP
#define DETECT_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <iomanip>
#include "inference.h"
#include <filesystem>
#include <fstream>
#include <random>


class DETECT {
public:

    // 运行目标检测
    void DetectTest(cv::Mat img);

    // 运行分类任务
    void RunClassification();

private:
    // YOLO 检测器智能指针
    std::unique_ptr<YOLO_V8> yoloDetector_;
    
    // 模型初始化参数
    DL_INIT_PARAM params_;


    // 读取 COCO 格式的 YAML 类别文件
    void ReadCocoYaml();
};

#endif // DETECT_HPP