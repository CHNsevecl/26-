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
            
            std::cout << "Delta Position: (" << delta_pos->x << ", " << delta_pos->y << ")" << std::endl;

            if (std::abs(delta_pos->x) <=10 && std::abs(delta_pos->y) <= 10 ){
                    if(finish_flag){
                        send_direction_to_MCU(uart, cv::Point(0, 0), 2, "RP5", "END");
                        return -1;
                    }
                    return 1;
            }
        
            if(start_flag){
                start_flag = false;
                send_direction_to_MCU(uart, delta_pos.value(), 1, "RP5", "END");
                // if(std::abs(delta_pos->x) < 2 && std::abs(delta_pos->y) < 2){
                    // send_direction_to_MCU(uart, delta_pos.value(), 0, "RP5", "END");
                // }
                
                
            }
            else{
                if(std::abs(delta_pos->x) < 3 && std::abs(delta_pos->y) < 3){
                    send_direction_to_MCU(uart, delta_pos.value(), 2, "RP5", "END");
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
        
        if(!data.has_value()){
        return 0;
        }
        for(const auto& point : data->Contours_vertex[data->target_index]){
            cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
        }
    return 0;
}


int Question3_Answer(UART& uart, UART& uart2, std::optional<ROI_with_oringin>& data0, cv::Mat& frame_BGR, cv::Mat& frame_binary){
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
            data0->Pre_Contours_vertex = data0->Contours_vertex;
            data0->Contours_vertex[data0->canvas_index] = target_range.value()[0];
            auto mapped_points = buildMappedTargetPoints(*data0);
            if (mapped_points.has_value()) {
                data0->object_vectors_on_canvas = mapped_points.value();
            }

            // data0 拼装完成后在画布上标记目标位置
            if(!data0->object_vectors_on_canvas.empty()){
                std::optional<cv::Point> delta_pos = std::nullopt;
                cv::Point Canvas_delta = cv::Point(0, 0);
                int Mission =3;

                if(!data0->Pre_Contours_vertex.empty()){
                    if(data0->Pre_Contours_vertex != data0->Contours_vertex){
                        Canvas_delta.x = data0->Contours_vertex[data0->canvas_index][0].x - data0->Pre_Contours_vertex[data0->canvas_index][0].x;
                        Canvas_delta.y = data0->Contours_vertex[data0->canvas_index][0].y - data0->Pre_Contours_vertex[data0->canvas_index][0].y;
                        
                    }
                    if(sqrt(pow(Canvas_delta.x, 2) + pow(Canvas_delta.y, 2)) >= 2){
                        Mission = 3;
                    }
                    else{
                        Mission = 30;

                    }
                }
                std::cout << "Canvas Delta: (" << Canvas_delta.x << ", " << Canvas_delta.y << ")" << "Mission: " << Mission << std::endl;
            

                if (data0->object_vectors_on_canvas.size() == 2) {
                    
                    delta_pos = delta_Position(data0, frame_BGR, data0->object_vectors_on_canvas[0]);
                    if(!delta_pos.has_value()){
                        return 0;
                    }
                    cv::circle(frame_BGR, data0->object_vectors_on_canvas[0], 5, cv::Scalar(0, 255, 0), -1);
                    
                }
                else if(data0->object_vectors_on_canvas.size() >= 3) {
                    cv::Point center;
                    for(const auto& point : data0->object_vectors_on_canvas){
                        center.x += point.x;
                        center.y += point.y;
                    }
                    center.x /= data0->object_vectors_on_canvas.size();
                    center.y /= data0->object_vectors_on_canvas.size();
                    
                    delta_pos = delta_Position(data0, frame_BGR, center);
                    if(!delta_pos.has_value()){
                        return 0;
                    }
                    cv::circle(frame_BGR, center, 5, cv::Scalar(0, 255, 0), -1);
                }

                
                if(sqrt(pow(delta_pos.value().x, 2) + pow(delta_pos.value().y, 2)) <= 1){
                    std::cout << "Target reached on canvas!" << std::endl;
                    info_to_MCU(uart2, {"START"});
                    Canvas_delta = cv::Point(0, 0);
                }
                send_direction_to_MCU(uart, delta_pos.value(),Mission, "RP5", "END");
                

            }
        }
    }
        
    return 0;

}

int Question4_Answer(UART& uart, UART& uart2, std::optional<ROI_with_oringin>& data0, cv::Mat& frame_BGR, cv::Mat& frame_binary, int drawed_points, bool& finish_flag){
    // ============ 阶段1：初始化（前50帧），像Q3一样稳定检测画布 ============
    if(!data0.has_value() || data0->count_for_q3 < 50){
        std::optional<ROI_with_oringin> data = find_object_positon_on_canvas(frame_BGR, frame_binary);
        if(!data.has_value()){
            return 0;
        }
        if(data->target_index != -1){
            if(data0.has_value()){
                data->count_for_q3 = data0->count_for_q3;
            }
            data0 = data;
            for(auto& point : data->Contours_vertex[data->target_index]){
                cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
            }
            data0->count_for_q3 = data0->count_for_q3 + 1;
        }
        return 0;
    }

    if(finish_flag){
        std::cout << "Q4 - All points drawn! Exiting." << std::endl;
        send_direction_to_MCU(uart, cv::Point(0, 0), 2, "RP5", "END");

        return -1;
    }

    // ============ 阶段2：像Q3一样持续跟踪画布位置变化 ============
    find_target_object(frame_BGR, frame_binary, 4, 1);
    std::vector<std::vector<cv::Point>> contours;
    findContours_and_Draw(frame_binary, frame_BGR, contours);
    std::optional<std::vector<std::vector<cv::Point>>> target_range = selectBestContour(contours, frame_BGR, 4, 1, true);
    
    if(!target_range.has_value()){
        return 0;
    }

    // 更新画布顶点
    target_range.value()[0] = orderRectanglePoints(target_range.value()[0]);
    data0->Pre_Contours_vertex = data0->Contours_vertex;
    data0->Contours_vertex[data0->canvas_index] = target_range.value()[0];
    
    // 根据新的画布位置重新映射目标点
    auto mapped_points = buildMappedTargetPoints(*data0);
    if(!mapped_points.has_value()){
        return 0;
    }
    data0->object_vectors_on_canvas = mapped_points.value();

    // ============ 计算画布位移决定Mission（同Q3逻辑） ============
    cv::Point Canvas_delta(0, 0);
    int Mission = 4;  // 默认纯画画（激光开）
    if(!data0->Pre_Contours_vertex.empty()){
        if(data0->Pre_Contours_vertex != data0->Contours_vertex){
            Canvas_delta.x = data0->Contours_vertex[data0->canvas_index][0].x - data0->Pre_Contours_vertex[data0->canvas_index][0].x;
            Canvas_delta.y = data0->Contours_vertex[data0->canvas_index][0].y - data0->Pre_Contours_vertex[data0->canvas_index][0].y;
        }
    }
    

    // ============ 像Q2一样逐点画画（完全复用Q2逻辑） ============
    const bool circle_mode = isCircleDescriptor(data0->target_absolute_position);
    std::vector<cv::Point> target_points = circle_mode
        ? buildCirclePath(data0->object_vectors_on_canvas, 36, 15.0)
        : data0->object_vectors_on_canvas;

    if(target_points.empty()){
        return 0;
    }

    const int target_point_count = static_cast<int>(target_points.size());
    std::cout << "Q4 - Drawed: " << drawed_points << "/" << target_point_count << std::endl;

    // 用static变量记录是否已发送START信号
    static bool first_point_start_sent = false;

    // 画完target_point_count+1个点时（最后一笔），设finish_flag=true
    if(drawed_points == 1 + static_cast<int>(target_point_count)){
        finish_flag = true;
    }

    // finish_flag已设，下一帧直接退出
    if(finish_flag){
        std::cout << "Q4 - All points drawn! Exiting." << std::endl;
        send_direction_to_MCU(uart, cv::Point(0, 0), 2, "RP5", "END");
        return -1;
    }

    // 防止索引越界：当drawed_points超出范围时取0（最后一笔回到第一个点）
    int safe_index = drawed_points;
    if(safe_index >= target_point_count){
        safe_index = 0;
    }

    // 判断是否是起始帧（需要先移动到第一个点，Mission=1）
    bool start_flag = (drawed_points == 0);

    cv::Point target_center_on_all_frame = target_points[safe_index];
    std::optional<cv::Point> delta_pos = delta_Position(data0, frame_BGR, target_center_on_all_frame);

    // 在画布上标记当前目标点（红色）
    cv::circle(frame_BGR, target_center_on_all_frame, 5, cv::Scalar(0, 0, 255), -1);

    if(!delta_pos.has_value()){
        return 0;
    }

    std::cout << "Delta: (" << delta_pos->x << ", " << delta_pos->y << ")" << std::endl;

    // ============ 完全使用Q2的画画逻辑 ============

    if (std::abs(delta_pos->x) <= 10 && std::abs(delta_pos->y) <= 10){
        // 到达当前目标点附近
        
        // 第一次到达第一个点时，通过uart2通知小车出发
        if(drawed_points == 0 && !first_point_start_sent){
            std::cout << "Q4 - First point reached! Sending START to car." << std::endl;
            info_to_MCU(uart2, {"START"});
            first_point_start_sent = true;
        }

        // 推进到下一个点
        drawed_points++;
        return 1;
    }

    // 根据Q2逻辑发送指令
    if(start_flag){
        // 第一次移动到目标点，用Mission=1（激光关，只移动）
        start_flag = false;
        send_direction_to_MCU(uart, delta_pos.value(), 1, "RP5", "END");
    }
    else{
        // 画点模式
        cv::Point cmd = delta_pos.value();
        
        if(std::abs(cmd.x) < 3 && std::abs(cmd.y) < 3){
            send_direction_to_MCU(uart, cmd, Mission, "RP5", "END");
        }
        else{
            double length = sqrt(pow(cmd.x, 2) + pow(cmd.y, 2));
            double sin_val = cmd.y / length;
            double cos_val = cmd.x / length;
            cv::Point D_delta_pos = cv::Point(int(5.0 * cos_val), int(5.0 * sin_val));
            D_delta_pos -= Canvas_delta * 4; // 画布在动时，补偿画布位移
            Mission = 4; // 画点模式（激光开）
            send_direction_to_MCU(uart, D_delta_pos, Mission, "RP5", "END");

            std::cout << "Q4 - Canvas Delta: (" << Canvas_delta.x << ", " << Canvas_delta.y << ") Mission: " << Mission << std::endl;
        }
    }

    // 画目标物体轮廓
    for(const auto& point : data0->Contours_vertex[data0->target_index]){
        cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
    }

    return 0;
}