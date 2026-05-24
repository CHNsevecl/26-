#include "Answer.hpp"

void Question1_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary){
    std::vector<int> lines = {0, 3, 4};
    std::optional<cv::Point> delta_pos;
    for (const auto& i : lines){
        std::optional<ROI_with_oringin> data = find_target_object(frame_BGR, frame_binary, i, 2);
        delta_pos = delta_Position(data, frame_BGR);
        if (delta_pos.has_value()){
            break;
        }
    }
    if (delta_pos.has_value()){
        send_direction_to_MCU(uart, delta_pos.value(), "RP5", "END");
    }
}
        