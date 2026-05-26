#ifndef Mystruct_HPP
#define Mystruct_HPP

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>


/// @brief 存储兴趣点数据的结构体
/// @param roi_vector 存储兴趣点所在的ROI矩形，包含兴趣点的最小矩形
/// @param object_ROI_data 存储兴趣点在ROI内的坐标，第一层vector是多个图形，第二层vector是每个图形的顶点坐标，如果没有找到符合条件的图形，则返回std::nullopt
/// @param Contours_vertex 存储兴趣点的顶点坐标，第一层vector是多个图形，第二层vector是每个图形的顶点坐标
/// @param target_index 存储目标图形在roi_vector中的索引，-1表示没有找到符合条件的图形
/// @note Contours_vertex[target_index]是目标图形的框的顶点坐标，如果target_index为-1，则Contours_vertex[target_index]无效
/// @note roi_vector[target_index]的数据结构是【左上 左下 右下 右上】
/// @note Contours_vertex和object_ROI_data的索引是一一对应的，即Contours_vertex[i]对应object_ROI_data[i]，如果Contours_vertex[i]没有找到符合条件的图形，则object_ROI_data[i]为std::nullopt
/// @note target_absolute_position存储目标图形在画布上的绝对坐标，第一层vector是每个图形的顶点坐标，如果没有找到符合条件的图形，则返回std::nullopt
struct ROI_with_oringin{
    std::vector<cv::Rect> roi_vector;
    std::vector<std::vector<cv::Point>> object_ROI_data;
    std::vector<std::vector<cv::Point>> Contours_vertex;
    std::vector<cv::Point> target_absolute_position;
    std::vector<cv::Point> object_vectors_on_canvas;

    int target_index = -1;
    int canvas_index = -1;
    int count_for_q3 = 0;
    int dx = 0;

};

struct Math_proccessing_data{
    double target_distance_to_upper_line;
    double tan_target_contour;
    double sin_target_contour;
    double cos_target_contour;
};

#endif