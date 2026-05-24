#include "Laser_Position.hpp"

std::optional<cv::Point> delta_Position(const std::optional<ROI_with_oringin>& Position_data, cv::Mat& frame_BGR){
    ROI_with_oringin object_data;
    if(Position_data.has_value()){
        object_data = Position_data.value();
    }

    int target_index = -1;
    cv::Mat frame_HSV;
    for (int i = 0; i < object_data.object_ROI_data.size(); i++){
        if(object_data.object_ROI_data[i][0] == cv::Point(-1, -1)){
            continue;
        }
        target_index = i;
        break;
    }
    if(target_index == -1){
        // std::cerr << "No valid target found" << std::endl;
        return std::nullopt;
    }
    
    cv::cvtColor(frame_BGR, frame_HSV, cv::COLOR_BGR2HSV);
    cv::Mat mask_blue;
    cv::inRange(frame_HSV, cv::Scalar(100, 30, 230), cv::Scalar(130, 255, 255), mask_blue);
    cv::Mat kernel_blue = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    // 只填充微小空洞，不去主动腐蚀
    cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel_blue);

    std::vector<cv::Point> all_point;
    cv::Point delta_Position;
    all_point.reserve(1);
    std::vector<std::vector<cv::Point>> contours_blue;
    cv::findContours(mask_blue, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours_blue) {
        cv::Moments m = cv::moments(contour);
        if (m.m00 > 0) {
            cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
            // center 就是激光中心
            cv::circle(frame_BGR, center,2,cv::Scalar(255,0,0),-1);//画点
            all_point.push_back(center);
            break;
        }
    }

    if (!all_point.empty()) {
        cv::Point target_center_on_all_frame;
        int Point_num = object_data.object_ROI_data[target_index].size();
        // std::cout << Point_num << std::endl;
        if(Point_num == 2){//圆的情况，只有两个点，直接取第一个点,因为第二个点是(Radius, 0)
            Point_num = 1;
            target_center_on_all_frame.x = (object_data.object_ROI_data[target_index][0].x ) + object_data.roi_vector[target_index].x;
            target_center_on_all_frame.y = (object_data.object_ROI_data[target_index][0].y ) + object_data.roi_vector[target_index].y;
        }
        else{
            // std::cout << "Point_num: " << Point_num << std::endl;
            for (int i =0;i<object_data.object_ROI_data[target_index].size();i++){ 
                if(object_data.object_ROI_data[target_index][i] == cv::Point(-1, -1)){
                    Point_num--;
                    continue;
                }
                target_center_on_all_frame.x += object_data.object_ROI_data[target_index][i].x;
                target_center_on_all_frame.y += object_data.object_ROI_data[target_index][i].y;
            }
            target_center_on_all_frame.x = target_center_on_all_frame.x / Point_num;
            target_center_on_all_frame.y = target_center_on_all_frame.y / Point_num;
            
            target_center_on_all_frame.x += object_data.roi_vector[target_index].x;
            target_center_on_all_frame.y += object_data.roi_vector[target_index].y;  
        }
        delta_Position.x = all_point.front().x - target_center_on_all_frame.x;
        delta_Position.y = all_point.front().y - target_center_on_all_frame.y;
        // std::cout << "Delta Position: (" << delta_Position.x << ", " << delta_Position.y << ")" << std::endl;
        
        return std::make_optional(delta_Position);
    }

    return std::nullopt;
}