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
        ? buildCirclePath(data->object_vectors_on_canvas, 36, 15.0)
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
                send_direction_to_MCU(uart, delta_pos.value(),1, "RP5", "END");
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

                    send_direction_to_MCU(uart, D_delta_pos,2, "RP5", "END");
                }
            }
        }
        
        if(!data.has_value()){
        return 0;
        }
        for(const auto& point : data->Contours_vertex[data->target_index]){
            cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
        }
    return 0;
}

int Question3_Answer(UART& uart, std::optional<ROI_with_oringin>& data0, cv::Mat& frame_BGR, cv::Mat& frame_binary){

    if(data0->count_for_q3 < 50){
        std::optional<ROI_with_oringin> data = find_object_positon_on_canvas(frame_BGR, frame_binary);
        if(!data.has_value()){
            return 0 ;
        }
        if(data->target_index != -1){
            data->count_for_q3 = data0->count_for_q3;
            data0 = data;
            for(auto& point : data->Contours_vertex[data->target_index]){
                cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
            }

            data0->count_for_q3 = data0->count_for_q3 + 1;
            
        }
    }
    else{
        //data0 拼装
        find_target_object(frame_BGR, frame_binary, 4, 1);
        std::vector<std::vector<cv::Point>> contours;
        findContours_and_Draw(frame_binary, frame_BGR, contours);
        std::optional<std::vector<std::vector<cv::Point>>> target_range = selectBestContour(contours, frame_BGR, 4, 1, true);
        
        if(target_range.has_value()){
            
            target_range.value()[0] = orderRectanglePoints(target_range.value()[0]);
            data0->Contours_vertex[data0->canvas_index] = target_range.value()[0];
            auto mapped_points = buildMappedTargetPoints(*data0);
            if (mapped_points.has_value()) {
                data0->object_vectors_on_canvas = mapped_points.value();
            }

            // data0 拼装完成后在画布上标记目标位置
            if(!data0->object_vectors_on_canvas.empty()){
                for(auto& point : data0->object_vectors_on_canvas){
                    cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
                }
                cv::circle(frame_BGR, data0->object_vectors_on_canvas[0], 5, cv::Scalar(0, 0, 255), -1);
                // cv::circle(frame_BGR, data0->object_vectors_on_canvas[1], 5, cv::Scalar(0, 255, 0), -1);
                if (data0->object_vectors_on_canvas.size() == 2) {
                    std::optional<cv::Point> delta_pos = delta_Position(data0, frame_BGR, data0->object_vectors_on_canvas[0]);
                    if (delta_pos.has_value()) {
                        int bias = data0->dx;
                        cv::Point corrected = delta_pos.value();
                        if (bias != 0) {
                            if (delta_pos->x >= 0) corrected += cv::Point(bias, 0);
                            else corrected -= cv::Point(bias, 0);
                        }

                        // 先发送包含偏置的主移动，再发送不带偏置的精调
                        send_direction_to_MCU(uart, corrected, 3, "RP5", "END");
                        send_direction_to_MCU(uart, delta_pos.value(), 3, "RP5", "END");

                        // 偏置衰减：按 10% 衰减，最小步长为 1
                        if (bias != 0) {
                            int sign = (bias > 0) ? 1 : -1;
                            int decay = std::max(1, std::abs(bias) / 10);
                            data0->dx = bias - sign * decay;
                        }
                    }
                }



            }
        }
    }

    

    
        
    return 0;
}
