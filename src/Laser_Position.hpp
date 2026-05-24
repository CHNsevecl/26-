#ifndef Laser_Position_HPP
#define Laser_Position_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include "mystruct.hpp"



std::optional<cv::Point> delta_Position(const std::optional<ROI_with_oringin>& Position_data, cv::Mat& frame_BGR);

#endif