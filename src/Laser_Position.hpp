#ifndef Laser_Position_HPP
#define Laser_Position_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include "mystruct.hpp"



std::optional<cv::Point> delta_Position(std::optional<ROI_with_oringin>& Position_data, cv::Mat& frame_BGR, cv::Point target_center_on_canvas);
cv::Point find_target_center_on_all_frame(ROI_with_oringin& object_data);

#endif