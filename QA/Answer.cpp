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
    // std::cout << "Object position on canvas: ";
    // for(const auto& point : data->object_vectors_on_canvas){
    //     std::cout << "(" << point.x << ", " << point.y << ") ";
    // }
    // std::cout << std::endl;


    // for (int i = 0; i < data->object_vectors_on_canvas.size(); i++){
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
                
            }
            else{
                send_direction_to_MCU(uart, delta_pos.value(), 2, "RP5", "END");
            }
        }
    
    // }
    return 0;
}