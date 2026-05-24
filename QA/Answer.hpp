#ifndef Answer_hpp
#define Answer_hpp

#include "../src/ImagePg.hpp"
#include "../src/Laser_Position.hpp"
#include "../src/uart.hpp"
#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

void Question1_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary);

#endif