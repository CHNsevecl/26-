#include "ImagePg.hpp"


/// @brief 形态学去噪
/// @param binary_frame // 二值图像 
/// @param kernel_size // 形态学核大小，默认为2
void morphologyProcess(cv::Mat& binary_frame, int kernel_size) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    
    cv::erode(binary_frame, binary_frame, kernel);
    cv::dilate(binary_frame, binary_frame, kernel);

    // 先闭运算：补小黑洞、连断裂边缘
    cv::morphologyEx(binary_frame, binary_frame, cv::MORPH_CLOSE, kernel);

    // 再开运算：去掉零散小噪声
    cv::morphologyEx(binary_frame, binary_frame, cv::MORPH_OPEN, kernel);
}

/// @brief 找轮廓
/// @param binary_frame // 二值图像
/// @param frame_BGR // 原始BGR图像
/// @param contours // 输出轮廓点集
/// @return 画面
cv::Mat findContours(const cv::Mat& binary_frame ,const cv::Mat& frame_BGR,std::vector<std::vector<cv::Point>>& contours) {
    
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat contour_vis = frame_BGR.clone();
    for (const auto& contour : contours){
        double area = cv::contourArea(contour);
        if (area < 500) continue;
        cv::drawContours(contour_vis, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
    }

    return contour_vis;
}

/// @brief 挑选最佳多变形
/// @param frame_BGR // 原始BGR图像
/// @param contours // 输出轮廓点集
/// @param lines // 期望的边数，默认为4(!!如果为0则挑选圆形)
/// @param nums //挑选出的图形数量
/// @param draw // 是否在画面上绘制最佳图形，默认为false
/// @return 最佳多边形顶点
std::optional<std::vector<std::vector<cv::Point>>> selectBestContour(const std::vector<std::vector<cv::Point>>& contours, cv::Mat& frame_BGR, int lines, int nums, bool draw) {
    std::vector<double> area_vector;
    std::vector<double> rad_vector;
    std::vector<std::vector<cv::Point>> best_rect;
    best_rect.reserve(nums); 
    area_vector.reserve(nums);
    rad_vector.reserve(nums);
    for(int i = 0;i<nums;i++){
        area_vector.push_back(0.0);
        rad_vector.push_back(0.0);
        best_rect.push_back(std::vector{cv::Point(0,0)});
    }
    

    if (lines == 0){//圆的情况
        
        for (const auto& contour : contours){
            double area = cv::contourArea(contour);
            if (area < 500) continue;
            double perimeter = cv::arcLength(contour, true);

            double rad = 4*3.14*area/perimeter/perimeter;
            if (rad > 0.8 && rad > rad_vector.back()){
                for(int j = 0; j<nums;j++){
                    if (rad_vector[j] < rad){
                        for(int i = nums-1;i>j;i--){
                            rad_vector[i] = rad_vector[i-1]; 
                            best_rect[i] = best_rect[i-1];
                        }
                        rad_vector[j] = rad;
                        best_rect[j] = contour;
                        break;
                    }
                }
            }
        }

        if(rad_vector[0] > 0){
            for(int i = 0;i <nums;i++){
                if (rad_vector[i] > 0){
                    if(draw){
                        cv::polylines(frame_BGR, best_rect[i], true, cv::Scalar(0, 0, 255), 2);
                    }
                    cv::Point2f center;
                    float radius = 0;
                    cv::minEnclosingCircle(best_rect[i], center, radius);
                    best_rect[i].clear();
                    best_rect[i].reserve(2);
                    best_rect[i].push_back(cv::Point(center.x, center.y));
                    best_rect[i].push_back(cv::Point(radius, 0));
                }
                else{
                    best_rect[i].clear();
                    best_rect[i].reserve(2);
                    best_rect[i].push_back(cv::Point(-1, -1));
                    best_rect[i].push_back(cv::Point(-1, 0));
                }
            }
        }
        else{
            return std::nullopt;
        }

    }
    else{//多边形

        for (const auto& contour : contours){
            double area = cv::contourArea(contour);
            
            if (area < 1000) continue;

            std::vector<cv::Point> hull;
            cv::convexHull(contour, hull);
                
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(hull, true), true);
            if (approx.size() != lines) continue;

            // 检查长宽比
            cv::Rect box = cv::boundingRect(approx);
            double aspect_ratio = (double)box.width / box.height;
            if (aspect_ratio < 0.4 || aspect_ratio > 2.0) continue;
            if (area > area_vector.back()){
                for(int j = 0; j<nums;j++){
                    if (area_vector[j] < area){
                        for(int i = nums-1;i>j;i--){
                            area_vector[i] = area_vector[i-1]; 
                            best_rect[i] = best_rect[i-1];
                        }
                        area_vector[j] = area;
                        best_rect[j] = approx;
                        break;
                    }
                }
            }
        }

        if (area_vector[0] > 0 ){
            if(draw){
                for(const auto& rect : best_rect){
                    cv::polylines(frame_BGR, rect, true, cv::Scalar(0, 0, 255), 2);
                }
            }

        }
        else{
            return std::nullopt;
        }
    }
    return std::make_optional(best_rect);
}   
        
/// @brief 通过兴趣点找物体
/// @param contours // 输出轮廓点集
/// @param frame_BGR // 原始BGR图像
/// @param frame_binary // 二值化图像
/// @return 圆的中心点、半径，或多边形的顶点
/// @return 返回struct ROI_with_oringin，包含兴趣点的ROI和物体数据,ROI的左上角和长宽，和物体在ROI的坐标结合获得实际坐标
std::optional<ROI_with_oringin> find_target_object(cv::Mat& frame_BGR, cv::Mat& frame_binary, int lines, int roi_nums){
    //1、找框（兴趣点）
    std::vector<std::vector<cv::Point>> contours;
    findContours(frame_binary, frame_BGR, contours);//寻找全图轮廓
    std::optional<std::vector<std::vector<cv::Point>>> target_range = selectBestContour(contours, frame_BGR, 4, roi_nums, true);
    // std::vector<cv::Rect> roi_vector;
    std::vector<cv::Rect> box_vector;
    // std::vector<std::vector<cv::Point>> object_data(roi_nums);
    ROI_with_oringin Position_data;
    Position_data.object_ROI_data.reserve(roi_nums);

    if (!target_range.has_value()) return std::nullopt;
    for(int i =0;i<roi_nums;i++){
        if (target_range.value()[i][0].x > 0){
            cv::Rect box = cv::boundingRect(target_range.value()[i]);
            box_vector.push_back(box);
            cv::Rect roi = cv::Rect(std::min(frame_BGR.cols - 1, box.x+10), std::min(frame_BGR.rows - 1, box.y+10), std::max(0, box.width-20), std::max(0, box.height-25));
            Position_data.roi_vector.push_back(roi);
        }
        else{
            box_vector.push_back(cv::Rect(0, 0, 0, 0));
            Position_data.roi_vector.push_back(cv::Rect(0, 0, 0, 0));
        }   
    }
    //2、找物体
    //contour_vis 修改到兴趣点
    for(int i = 0;i<roi_nums;i++){
        contours.clear();
        findContours(frame_binary(Position_data.roi_vector[i]), frame_BGR(Position_data.roi_vector[i]), contours);
        cv::Mat frame_BGRROI = frame_BGR(Position_data.roi_vector[i]);
        auto data = selectBestContour(contours, frame_BGRROI, lines, 1, true);
        
        if(data.has_value()){
            Position_data.object_ROI_data.push_back(data.value()[0]);
        }
        else{
            Position_data.object_ROI_data.push_back(std::vector{cv::Point(-1, -1)});
        }
    }
    
    return std::make_optional(Position_data);
}

//三角形
        // std::optional<ROI_with_oringin> object_data = find_target_object(contours, frame_BGR, frame_binary, 3, 2);
        // if (object_data.has_value()){
        //     for(int i =0;i<2;i++){
        //         if(object_data.value().object_ROI_data[i][0] != cv::Point(-1, -1)){
        //             for(const auto& point : object_data.value().object_ROI_data[i]){
        //                 std::cout << "(" << point.x + object_data.value().roi_vector[i].x << ", " << point.y + object_data.value().roi_vector[i].y << ")" << std::endl;
        //             }
        //         }
        //     }
        // }

        //矩形
        // std::optional<ROI_with_oringin> object_data = find_target_object(contours, frame_BGR, frame_binary, 4, 2);
        // if (object_data.has_value()){
        //     for(int i =0;i<2;i++){
        //         if(object_data.value().object_data[i][0] != cv::Point(-1, -1)){
        //             for(const auto& point : object_data.value().object_data[i]){
        //                 std::cout << "(" << point.x + object_data.value().roi_vector[i].x << ", " << point.y + object_data.value().roi_vector[i].y << ")" << std::endl;
        //             }
        //             std::cout << std::endl;
        //         }
        //     }
        // }
        //圆形
        // std::optional<ROI_with_oringin> object_data = find_target_object(contours, frame_BGR, frame_binary, 0, 2);
        // if (object_data.has_value()){
        //     for(int i =0;i<2;i++){
        //         if(object_data.value().object_data[i][0] != cv::Point(-1, -1)){
                    
        //             std::cout << "(" << object_data.value().object_data[i][0].x + object_data.value().roi_vector[i].x << ", " << object_data.value().object_data[i][0].y + object_data.value().roi_vector[i].y << ")" << std::endl;
                    
        //             std::cout << "Radius:" << object_data.value().object_data[i][1].x;
        //             std::cout << std::endl;
        //         }
        //     }
        // }