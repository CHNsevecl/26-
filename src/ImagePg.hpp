#ifndef ImagePg_HPP
#define ImagePg_HPP

#include <opencv2/opencv.hpp>
#include <optional>
#include "mystruct.hpp"

void morphologyProcess(cv::Mat& binary_frame, int kernel_size = 2);
cv::Mat findContours_and_Draw(const cv::Mat& binary_frame ,const cv::Mat& frame_BGR,std::vector<std::vector<cv::Point>>& contours);
std::optional<std::vector<std::vector<cv::Point>>> selectBestContour(const std::vector<std::vector<cv::Point>>& contours, cv::Mat& frame_BGR, int lines = 4, int nums = 1, bool draw = false);
bool isCircleDescriptor(const std::vector<cv::Point>& points);
std::vector<cv::Point> buildCirclePath(const std::vector<cv::Point>& mapped_points, int sample_count = 36, double per_pixel_length = 15.0);
std::optional<ROI_with_oringin> find_target_object( cv::Mat& frame_BGR, cv::Mat& frame_binary, int lines = 0, int roi_nums = 2);
std::optional<ROI_with_oringin> find_object_positon_on_canvas(cv::Mat& frame_BGR, cv::Mat& frame_binary);
void Distance_to_line_translator(ROI_with_oringin& data, std::vector<cv::Point>& object_Relate_to_Contours_vectors, std::vector<cv::Point>& object_vectors_on_canvas, int& contours_index);
std::vector<cv::Point> orderRectanglePoints(const std::vector<cv::Point>& points);
std::optional<std::vector<cv::Point>> buildMappedTargetPoints(const ROI_with_oringin& data);




#endif