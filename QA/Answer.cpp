#include "Answer.hpp"

#include <cmath>

namespace {

cv::Point contourCenter(const std::vector<cv::Point>& points) {
    cv::Point center(0, 0);
    if (points.empty()) {
        return center;
    }

    for (const auto& point : points) {
        center += point;
    }

    center.x /= static_cast<int>(points.size());
    center.y /= static_cast<int>(points.size());
    return center;
}

bool isZeroPoint(const cv::Point& point) {
    return point.x == 0 && point.y == 0;
}

cv::Point2d toPoint2d(const cv::Point& point) {
    return cv::Point2d(static_cast<double>(point.x), static_cast<double>(point.y));
}

cv::Point toPoint(const cv::Point2d& point) {
    return cv::Point(cvRound(point.x), cvRound(point.y));
}

cv::Point centerDelta(const std::vector<cv::Point>& old_points, const std::vector<cv::Point>& new_points) {
    if (old_points.empty() || new_points.empty()) {
        return cv::Point(0, 0);
    }

    return contourCenter(new_points) - contourCenter(old_points);
}

}

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

int Question3_Answer(UART& uart, UART& uart2,std::optional<ROI_with_oringin>& data0, cv::Mat& frame_BGR, cv::Mat& frame_binary){
    static cv::Point2d canvas_velocity_px_per_sec(0.0, 0.0);
    static bool canvas_velocity_valid = false;
    constexpr double frame_interval_sec = 1.0 / 30.0;

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
        cv::Point Canvas_Position_delta = cv::Point(0, 0);
        //data0 拼装
        find_target_object(frame_BGR, frame_binary, 4, 1);
        std::vector<std::vector<cv::Point>> contours;
        findContours_and_Draw(frame_binary, frame_BGR, contours);
        std::optional<std::vector<std::vector<cv::Point>>> target_range = selectBestContour(contours, frame_BGR, 4, 1, true);
        
        if(target_range.has_value()){
            
            target_range.value()[0] = orderRectanglePoints(target_range.value()[0]);
            Canvas_Position_delta = centerDelta(data0->Contours_vertex[data0->canvas_index], target_range.value()[0]);

            if (!isZeroPoint(Canvas_Position_delta)) {
                const cv::Point2d current_velocity_px_per_sec = toPoint2d(Canvas_Position_delta) / frame_interval_sec;
                if (!canvas_velocity_valid) {
                    canvas_velocity_px_per_sec = current_velocity_px_per_sec;
                    canvas_velocity_valid = true;
                }
                else {
                    canvas_velocity_px_per_sec = (canvas_velocity_px_per_sec + current_velocity_px_per_sec) * 0.5;
                }
            }

            data0->Contours_vertex[data0->canvas_index] = target_range.value()[0];
            auto mapped_points = buildMappedTargetPoints(*data0);
            if (mapped_points.has_value()) {
                data0->object_vectors_on_canvas = mapped_points.value();

                
            }
            std::cout << "Canvas Position Delta: (" << Canvas_Position_delta.x << ", " << Canvas_Position_delta.y << ")" << std::endl;

            // data0 拼装完成后在画布上标记目标位置
            if(!data0->object_vectors_on_canvas.empty()){
                for(auto& point : data0->object_vectors_on_canvas){
                    cv::circle(frame_BGR, point, 5, cv::Scalar(0, 255, 0), -1);
                }
                
                // cv::circle(frame_BGR, data0->object_vectors_on_canvas[1], 5, cv::Scalar(0, 255, 0), -1);
                if (data0->object_vectors_on_canvas.size() == 2) {
                    std::optional<cv::Point> delta_pos = delta_Position(data0, frame_BGR, data0->object_vectors_on_canvas[0]);
                    if (delta_pos.has_value()) {
                        const cv::Point canvas_compensation = isZeroPoint(Canvas_Position_delta)
                            ? cv::Point(0, 0)
                            : toPoint(canvas_velocity_px_per_sec * frame_interval_sec);
                        send_direction_to_MCU(uart, delta_pos.value() + canvas_compensation, 3,"RP5", "END");
                    }
                }
                else {
                    cv::Point center = cv::Point(0, 0);
                    for(const auto& point : data0->object_vectors_on_canvas){
                        center += point;   
                    }
                    center.x /= data0->object_vectors_on_canvas.size();
                    center.y /= data0->object_vectors_on_canvas.size();
                    std::optional<cv::Point> delta_pos = delta_Position(data0, frame_BGR, center);
                    // cv::circle(frame_BGR, center, 5, cv::Scalar(0, 0, 255), -1);、
                    if(delta_pos.has_value()){
                        std::cout << "Delta Position for Q3: (" << delta_pos->x << ", " << delta_pos->y << ")" << std::endl;
                        const cv::Point canvas_compensation = isZeroPoint(Canvas_Position_delta)
                            ? cv::Point(0, 0)
                            : toPoint(canvas_velocity_px_per_sec * frame_interval_sec);
                        if(data0->count_for_q3 == 50){
                            if(abs(delta_pos->x) <= 10 && abs(delta_pos->y) <= 10){
                                data0->count_for_q3 = data0->count_for_q3 + 1;
                                info_to_MCU(uart2, {"START"});
                            }
                            send_direction_to_MCU(uart, delta_pos.value() + canvas_compensation, 30,"RP5", "END");

                        }
                        // info_to_MCU(uart2, {"START"});
                        // if(abs(delta_pos->x) >= 2 || abs(delta_pos->y) >= 2){
                        else{
                            send_direction_to_MCU(uart, delta_pos.value() + canvas_compensation, 3,"RP5", "END");
                        }
                        // }
                        std::cout << "Canvas Compensation: (" << canvas_compensation.x << ", " << canvas_compensation.y << ")" << std::endl;
                        
                    }
                    
                }



            }
        }
    }

    

    
        
    return 0;
}
