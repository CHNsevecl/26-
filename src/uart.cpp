#include "uart.hpp"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>



void info_to_MCU(UART& uart, const std::vector<std::string>& info) {

    std::string messages = "";
    for (size_t i = 0; i < info.size(); i++) {
        if (i > 0){messages += " ";}
        messages += info[i];
    }
    
    messages = messages + "\n";
    uart.send(messages);
    usleep(2500); // 2.5ms
}

void send_direction_to_MCU(UART& uart,const cv::Point& direction,std::string start_bytes,std::string end_bytes) {
    vector<std::string> messages;
    messages.reserve(4);
    messages.push_back(start_bytes);
    messages.push_back(std::to_string(direction.x));
    messages.push_back(std::to_string(direction.y));
    messages.push_back(end_bytes);
    info_to_MCU(uart, messages);
}