#include "Answer.hpp"

#include <cmath>

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

    const bool circle_mode = isCircleDescriptor(data->target_absolute_position);
    std::vector<cv::Point> target_points = circle_mode
        ? buildCirclePath(data->object_vectors_on_canvas)
        : data->object_vectors_on_canvas;

    if (target_points.empty()) {
        return 0;
    }

    const int target_point_count = static_cast<int>(target_points.size());

        std::cout << "Drawed points: " << drawed_points << std::endl;
        if(drawed_points >= target_point_count){
            drawed_points = 0;
            finish_flag = true;
        }
        if(!finish_flag && drawed_points == 0){
            start_flag = true;
        }
        cv::Point target_center_on_all_frame = target_points[drawed_points];
        std::optional<cv::Point> delta_pos = delta_Position(data, frame_BGR, target_center_on_all_frame);

        if (circle_mode && data->object_vectors_on_canvas.size() >= 2) {
            const cv::Point circle_center = data->object_vectors_on_canvas[0];
            const int circle_radius = static_cast<int>(cv::norm(data->object_vectors_on_canvas[0] - data->object_vectors_on_canvas[1]));
            // if (circle_radius > 0) {
            //     cv::circle(frame_BGR, circle_center, circle_radius, cv::Scalar(0, 255, 0), 2);
            // }

            // std::cout << "circle : " << circle_center.x << ", " << circle_center.y << " radius: " << circle_radius << std::endl;
        }

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

int Question3_Answer(UART& uart, cv::Mat& frame_BGR, cv::Mat& frame_binary){
    std::vector<int> lines = {0, 3, 4};
    std::optional<cv::Point> delta_pos;

    std::vector<std::string> message = uart.receive();
    // if(message.size() >= 4){
    //     if (message[0] == "PICO2" && message[1] == "0" && message[2] == "0" && message[3] == "END"){
    //         return 0;
    //     }
    // }
    
    for (const auto& i : lines){
        std::optional<ROI_with_oringin> data = find_object_positon_on_canvas(frame_BGR, frame_binary);
        if(!data.has_value()){
            continue;
            
        }
        if (data->target_absolute_position.empty()) {
            continue;
        }

        cv::Point target_center_on_all_frame ;
        for(const auto& point : data->object_vectors_on_canvas){
            target_center_on_all_frame.x += point.x;
            target_center_on_all_frame.y += point.y;
        }
        target_center_on_all_frame.x /= static_cast<int>(data->object_vectors_on_canvas.size());
        target_center_on_all_frame.y /= static_cast<int>(data->object_vectors_on_canvas.size());
        
        delta_pos = delta_Position(data, frame_BGR, target_center_on_all_frame);
            if (delta_pos.has_value()){
                break;
            }
        }
        if (delta_pos.has_value()){
            send_direction_to_MCU(uart, delta_pos.value(), 3, "RP5", "END");
        }
    return 0;
}