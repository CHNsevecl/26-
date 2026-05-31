#include "Laser_Position.hpp"

#include <limits>

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
    
    // 单层阈值：只保留激光高亮核心
    cv::Mat mask_blue;
    cv::inRange(frame_HSV, cv::Scalar(100, 30, 220), cv::Scalar(130, 255, 255), mask_blue);
    
    // 形态学处理：小核膨胀连接断裂，闭运算填洞
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::dilate(mask_blue, mask_blue, kernel);
    cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_CLOSE, kernel);

    struct LaserCandidate {
        cv::Point2f center;
        double score = 0.0;
        double area = 0.0;
        double circularity = 0.0;
    };

    std::vector<LaserCandidate> candidates;
    cv::Point delta_Position;
    std::vector<std::vector<cv::Point>> contours_blue;
    cv::findContours(mask_blue, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::imshow("Blue Mask", mask_blue);

    static bool has_last_center = false;
    static cv::Point2f last_center(0.0f, 0.0f);

    const double max_jump_distance = 60.0;

    for (const auto& contour : contours_blue) {
        cv::Moments m = cv::moments(contour);
        double area = cv::contourArea(contour);
        double perimeter = cv::arcLength(contour, true);
        if (m.m00 <= 0 || area < 50.0 || perimeter <= 0.0) {
            continue;
        }

        double circularity = 4.0 * CV_PI * area / (perimeter * perimeter);
        if (circularity < 0.15) {
            continue;
        }

        cv::Point2f center(m.m10 / m.m00, m.m01 / m.m00);
        candidates.push_back({center, circularity, area, circularity});  // score = circularity（取最圆的）
    }

    if (!candidates.empty()) {
        int best_index = -1;

        // 优先选离上一帧最近的（追踪连续性）
        if (has_last_center) {
            double min_dist = max_jump_distance;
            for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
                double distance = cv::norm(candidates[i].center - last_center);
                if (distance < min_dist) {
                    min_dist = distance;
                    best_index = i;
                }
            }
        }

        // 没有历史或没找到近的点，选最圆的
        if (best_index == -1) {
            double best_circularity = -1.0;
            for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
                if (candidates[i].circularity > best_circularity) {
                    best_circularity = candidates[i].circularity;
                    best_index = i;
                }
            }
        }

        const cv::Point2f center = candidates[best_index].center;
        last_center = center;
        has_last_center = true;

        cv::circle(frame_BGR, center, 2, cv::Scalar(255, 0, 0), -1);//画点

        delta_Position.x = static_cast<int>(std::round(center.x)) - target_center_on_all_frame.x;
        delta_Position.y = static_cast<int>(std::round(center.y)) - target_center_on_all_frame.y;
        
        // std::cout << "Delta Position: (" << delta_Position.x << ", " << delta_Position.y << ")" << std::endl;
        
        return std::make_optional(delta_Position);
    }

    return std::nullopt;
}



cv::Point find_target_center_on_all_frame(ROI_with_oringin& object_data){
    cv::Point target_center_on_all_frame(0, 0);
    if (object_data.target_index < 0 || object_data.target_index >= static_cast<int>(object_data.object_ROI_data.size())) {
        return target_center_on_all_frame;
    }
    if (object_data.target_absolute_position.empty()) {
        return target_center_on_all_frame;
    }

    int Point_num = static_cast<int>(object_data.object_ROI_data[object_data.target_index].size());
    // 圆的情况，只有两个点，直接取第一个点（圆心），第二个点是(Radius, 0)
    if(Point_num == 2){
        target_center_on_all_frame.x = object_data.target_absolute_position[0].x;
        target_center_on_all_frame.y = object_data.target_absolute_position[0].y;
    }
    else{
        for(const auto& point : object_data.target_absolute_position){
            target_center_on_all_frame.x += point.x;
            target_center_on_all_frame.y += point.y;
        }
        target_center_on_all_frame.x /= Point_num;
        target_center_on_all_frame.y /= Point_num;
    }
    
    return target_center_on_all_frame;
}