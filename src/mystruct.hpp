#ifndef Mystruct_HPP
#define Mystruct_HPP

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

struct ROI_with_oringin{
    std::vector<cv::Rect> roi_vector;
    std::vector<std::vector<cv::Point>> object_ROI_data;
};

#endif