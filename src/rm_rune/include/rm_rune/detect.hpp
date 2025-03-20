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

    void Detector(YOLO_V8*& p , cv::Mat img, cv::Mat result_img, std::vector<std::vector<cv::Point2f>>& contours);

    int ReadCocoYaml(YOLO_V8*& p);

    void DetectTest(YOLO_V8* yoloDetector);
};

#endif // DETECT_HPP