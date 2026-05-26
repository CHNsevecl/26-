#include "Answer.hpp"

int Question1_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary){
    std::vector<int> lines = {0, 3, 4};
    std::optional<cv::Point> delta_pos;

    std::vector<std::string> message = uart.receive();
    if(message.size() >= 4){
        if (message[0] == "PICO2" && message[1] == "0" && message[2] == "0" && message[3] == "END"){
            return -1;
        }
    }
    
    for (const auto& i : lines){
        std::optional<ROI_with_oringin> data = find_target_object(frame_BGR, frame_binary, i, 2);
        if(!data.has_value()){
            // if (data->target_index != -1){
            //     for(const auto& point : data->Contours_vertex[data->target_index]){
            //         std::cout << "(" << point.x << ", " << point.y << ") ";
            //     }
            //     std::cout << std::endl;
            // }
            continue;
            
        }
        if (data->target_absolute_position.empty()) {
            continue;
        }
        cv::Point target_center_on_all_frame = find_target_center_on_all_frame(data.value());
        delta_pos = delta_Position(data, frame_BGR, target_center_on_all_frame);
        if (delta_pos.has_value()){
            break;
        }
    }
    if (delta_pos.has_value()){
        send_direction_to_MCU(uart, delta_pos.value(), 1, "RP5", "END");
    }
    return 0;
}
        


int Question2_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary, int drawed_points){
    bool finish_flag = false;
    bool start_flag = false;
    
    std::optional<ROI_with_oringin> data = find_object_positon_on_canvas(frame_BGR, frame_binary);

    if(!data.has_value()){
        return 0 ;
    }

        std::cout << "Drawed points: " << drawed_points << std::endl;
        if(drawed_points == data->object_vectors_on_canvas.size()){
            drawed_points = 0;
            finish_flag = true;
        }
        if(!finish_flag && drawed_points == 0){
            start_flag = true;
        }
        cv::Point target_center_on_all_frame = data->object_vectors_on_canvas[drawed_points];
        std::optional<cv::Point> delta_pos = delta_Position(data, frame_BGR, target_center_on_all_frame);

        std::vector<std::string> message = uart.receive();

        if (delta_pos.has_value()){
            

            if(message.size() >= 4){
                if (message[0] == "PICO2" && message[1] == "0" && message[2] == "0" && message[3] == "END"){
                    if(finish_flag){
                        return -1;
                    }
                    return 1;
                }
            }
            if(start_flag){
                start_flag = false;
                send_direction_to_MCU(uart, delta_pos.value(), 1, "RP5", "END");
                if(abs(delta_pos->x) < 2 && abs(delta_pos->y) < 2){
                    send_direction_to_MCU(uart, delta_pos.value(), 0, "RP5", "END");
                }
                
                
            }
            else{
                if(abs(delta_pos->x) < 3 && abs(delta_pos->y) < 3){
                    send_direction_to_MCU(uart, delta_pos.value(), 0, "RP5", "END");
                }
                else{
                    double length = sqrt(pow(delta_pos->x, 2) + pow(delta_pos->y, 2));
                    double sin = delta_pos->y / length;
                    double cos = delta_pos->x / length;
                    cv::Point D_delta_pos = cv::Point(int(5.0*cos), int(5.0*sin));

                    send_direction_to_MCU(uart, D_delta_pos, 2, "RP5", "END");
                }
            }
        }
    return 0;
}