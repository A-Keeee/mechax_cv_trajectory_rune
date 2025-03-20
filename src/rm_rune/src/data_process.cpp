//预测主函数

#include <random>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <vector>

#include "rm_rune/contour_info.hpp"
#include "rm_rune/inference.h"
#include "rm_rune/detect.hpp"
#include "rm_rune/data_process.hpp"



namespace fs = std::filesystem;

ContourInfo::ContourInfo()
{
    // 空的构造函数，用于创建一个空对象
}

ContourInfo::ContourInfo(const std::vector<cv::Point2f>& contour)//contour改为存储标签置信度+五个关键点
{
    // 分别累加 x 和 y 坐标
    int sum_x = contour[1].x + contour[2].x + contour[5].x + contour[4].x;
    int sum_y = contour[1].y + contour[2].y + contour[5].y + contour[4].y;

    // 计算平均值
    this->center = cv::Point(sum_x / 4, sum_y / 4);  // 使用 cv::Point 初始化中心点
    this->circle_center = cv::Point(contour[3].x, contour[3].y);  // 使用 cv::Point 初始化圆心
    this->index = contour[0].x;  // 初始化索引
    this->conf = contour[0].y;  // 初始化置信度
}

// 设置轮廓并更新轮廓相关信息
void ContourInfo::setContour(const std::vector<cv::Point2f>& contour)
{
    // 分别累加 x 和 y 坐标
    int sum_x = contour[1].x + contour[2].x + contour[5].x + contour[4].x;
    int sum_y = contour[1].y + contour[2].y + contour[5].y + contour[4].y;

    // 计算平均值
    this->center = cv::Point2f(sum_x / 4, sum_y / 4);  // 使用 cv::Point 更新中心点
    this->circle_center = cv::Point(contour[3].x, contour[3].y);  // 使用 cv::Point 更新圆心
    this->index = contour[0].x;  // 更新索引
    this->conf = contour[0].y;  // 更新置信度
}


// Define the skeleton and color mappings
std::vector<std::vector<int>> skeleton = {{16, 14}, {14, 12}, {17, 15}, {15, 13}, {12, 13}, {6, 12}, {7, 13}, {6, 7},
                                          {6, 8}, {7, 9}, {8, 10}, {9, 11}, {2, 3}, {1, 2}, {1, 3}, {2, 4}, {3, 5}, {4, 6}, {5, 7}};

std::vector<cv::Scalar> posePalette = {
        cv::Scalar(255, 128, 0), cv::Scalar(255, 153, 51), cv::Scalar(255, 178, 102), cv::Scalar(230, 230, 0), cv::Scalar(255, 153, 255),
        cv::Scalar(153, 204, 255), cv::Scalar(255, 102, 255), cv::Scalar(255, 51, 255), cv::Scalar(102, 178, 255), cv::Scalar(51, 153, 255),
        cv::Scalar(255, 153, 153), cv::Scalar(255, 102, 102), cv::Scalar(255, 51, 51), cv::Scalar(153, 255, 153), cv::Scalar(102, 255, 102),
        cv::Scalar(51, 255, 51), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 0, 0), cv::Scalar(255, 255, 255)
};

std::vector<int> limbColorIndices = {9, 9, 9, 9, 7, 7, 7, 0, 0, 0, 0, 0, 16, 16, 16, 16, 16, 16, 16};
std::vector<int> kptColorIndices = {16, 16, 16, 16, 16, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9};



cv::Scalar generateRandomColor(int numChannels) {
    if (numChannels < 1 || numChannels > 3) {
        throw std::invalid_argument("Invalid number of channels. Must be between 1 and 3.");
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 255);

    cv::Scalar color;
    for (int i = 0; i < numChannels; i++) {
        color[i] = dis(gen); // for each channel separately generate value
    }

    return color;
}

std::vector<cv::Scalar> generateRandomColors(int class_names_num, int numChannels) {
    std::vector<cv::Scalar> colors;
    for (int i = 0; i < class_names_num; i++) {
        cv::Scalar color = generateRandomColor(numChannels);
        colors.push_back(color);
    }
    return colors;
}

