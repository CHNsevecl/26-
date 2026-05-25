#include "Laser_Position.hpp"

std::optional<cv::Point> delta_Position(std::optional<ROI_with_oringin>& Position_data, cv::Mat& frame_BGR, cv::Point target_center_on_all_frame){
    ROI_with_oringin object_data;
    if(Position_data.has_value()){
        object_data = Position_data.value();
    }

    cv::Mat frame_HSV;
    
    if(object_data.target_index == -1){
        // std::cerr << "No valid target found" << std::endl;
        return std::nullopt;
    }
    
    cv::cvtColor(frame_BGR, frame_HSV, cv::COLOR_BGR2HSV);
    cv::Mat mask_blue;
    cv::inRange(frame_HSV, cv::Scalar(100, 30, 150), cv::Scalar(130, 255, 255), mask_blue);
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

        delta_Position.x = all_point[0].x - target_center_on_all_frame.x;
        delta_Position.y = all_point[0].y - target_center_on_all_frame.y;
        
        // std::cout << "Delta Position: (" << delta_Position.x << ", " << delta_Position.y << ")" << std::endl;
        
        return std::make_optional(delta_Position);
    }

    return std::nullopt;
}



cv::Point find_target_center_on_all_frame(ROI_with_oringin& object_data){
    cv::Point target_center_on_all_frame;
    int Point_num = object_data.object_ROI_data[object_data.target_index].size();
    // std::cout << Point_num << std::endl;
    if(Point_num == 2){//圆的情况，只有两个点，直接取第一个点,因为第二个点是(Radius, 0)
        Point_num = 1;
        target_center_on_all_frame.x = object_data.target_absolute_position[0].x;
        target_center_on_all_frame.y = object_data.target_absolute_position[0].y;
    }
    else{
        // std::cout << "Point_num: " << Point_num << std::endl;
        for(const auto& point : object_data.target_absolute_position){
            target_center_on_all_frame.x += point.x;
            target_center_on_all_frame.y += point.y;
        }
        target_center_on_all_frame.x /= Point_num;
        target_center_on_all_frame.y /= Point_num;
    }
    
    return target_center_on_all_frame;
}