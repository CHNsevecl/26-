#ifndef ImagePg_HPP
#define ImagePg_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include "mystruct.hpp"

void morphologyProcess(cv::Mat& binary_frame, int kernel_size = 2);
cv::Mat findContours_and_Draw(const cv::Mat& binary_frame ,const cv::Mat& frame_BGR,std::vector<std::vector<cv::Point>>& contours);
std::optional<std::vector<std::vector<cv::Point>>> selectBestContour(const std::vector<std::vector<cv::Point>>& contours, cv::Mat& frame_BGR, int lines = 4, int nums = 1, bool draw = false);
std::optional<ROI_with_oringin> find_target_object( cv::Mat& frame_BGR, cv::Mat& frame_binary, int lines = 0, int roi_nums = 2);
std::optional<ROI_with_oringin> find_object_positon_on_canvas(cv::Mat& frame_BGR, cv::Mat& frame_binary);
void Distance_to_line_translator(ROI_with_oringin& data, std::vector<cv::Point>& object_Relate_to_Contours_vectors, std::vector<cv::Point>& object_vectors_on_canvas, int& contours_index);




#endif