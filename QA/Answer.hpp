#ifndef Answer_hpp
#define Answer_hpp

#include "../src/ImagePg.hpp"
#include "../src/Laser_Position.hpp"
#include "../src/uart.hpp"
#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

int Question1_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary);
int Question2_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary, int drawed_points);

#endif