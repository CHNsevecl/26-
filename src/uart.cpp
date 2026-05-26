#include "uart.hpp"
#include <algorithm>
#include <cctype>
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

void send_direction_to_MCU(UART& uart,const cv::Point& direction, int question_id,std::string start_bytes,std::string end_bytes) {
    vector<std::string> messages;
    messages.reserve(5);
    messages.push_back(start_bytes);
    messages.push_back(std::to_string(direction.x));
    messages.push_back(std::to_string(direction.y));

    messages.push_back(std::to_string(question_id));
    messages.push_back(end_bytes);
    info_to_MCU(uart, messages);
}

std::vector<std::string> UART::receive() {
    if (fd == -1) {
        return {};
    }

    std::string data;
    char buffer[256];

    while (true) {
        int available = 0;
        if (ioctl(fd, FIONREAD, &available) == -1 || available <= 0) {
            break;
        }

        int chunk_size = std::min(available, static_cast<int>(sizeof(buffer)));
        ssize_t length = read(fd, buffer, chunk_size);
        if (length > 0) {
            data.append(buffer, length);
        }

        if (length <= 0 || length < chunk_size) {
            break;
        }
    }

    std::vector<std::string> result;
    std::string token;

    for (unsigned char ch : data) {
        if (std::isalnum(ch) || ch == '_') {
            token.push_back(static_cast<char>(ch));
            continue;
        }

        if (!token.empty()) {
            result.push_back(token);
            token.clear();
        }
    }

    if (!token.empty()) {
        result.push_back(token);
    }

    return result;
}