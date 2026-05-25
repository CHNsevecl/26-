#include "ImagePg.hpp"
#include <algorithm>

static std::vector<cv::Point> orderRectanglePoints(const std::vector<cv::Point>& points) {
    std::vector<cv::Point> ordered(4);

    int top_left_index = 0;
    int top_right_index = 0;
    int bottom_left_index = 0;
    int bottom_right_index = 0;

    int min_sum = points[0].x + points[0].y;
    int max_sum = min_sum;
    int min_diff = points[0].x - points[0].y;
    int max_diff = min_diff;

    for (int i = 1; i < 4; ++i) {
        const int sum = points[i].x + points[i].y;
        const int diff = points[i].x - points[i].y;

        if (sum < min_sum) {
            min_sum = sum;
            top_left_index = i;
        }
        if (sum > max_sum) {
            max_sum = sum;
            bottom_right_index = i;
        }
        if (diff < min_diff) {
            min_diff = diff;
            top_right_index = i;
        }
        if (diff > max_diff) {
            max_diff = diff;
            bottom_left_index = i;
        }
    }

    ordered[0] = points[top_left_index];
    ordered[1] = points[top_right_index];
    ordered[2] = points[bottom_right_index];
    ordered[3] = points[bottom_left_index];

    return ordered;
}


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
cv::Mat findContours_and_Draw(const cv::Mat& binary_frame ,const cv::Mat& frame_BGR,std::vector<std::vector<cv::Point>>& contours) {
    
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
/// @return 最佳多边形顶点。第一个vector是多个图形，第二个vector是每个图形的顶点坐标。如果没有找到符合条件的图形，则返回std::nullopt
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
                        if(lines == 4) best_rect[j] = orderRectanglePoints(approx);
                        else best_rect[j] = approx;
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
    return std::make_optional(best_rect);//返回轮廓近似的多边形的几个点
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
    ROI_with_oringin Position_data;
    findContours_and_Draw(frame_binary, frame_BGR, contours);//寻找全图轮廓
    std::optional<std::vector<std::vector<cv::Point>>> target_range = selectBestContour(contours, frame_BGR, 4, roi_nums, true);
    std::vector<cv::Rect> box_vector;
    Position_data.object_ROI_data.reserve(roi_nums);

    if (!target_range.has_value()) return std::nullopt;
    Position_data.Contours_vertex = target_range.value();
    
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
        findContours_and_Draw(frame_binary(Position_data.roi_vector[i]), frame_BGR(Position_data.roi_vector[i]), contours);
        cv::Mat frame_BGRROI = frame_BGR(Position_data.roi_vector[i]);
        auto data = selectBestContour(contours, frame_BGRROI, lines, 1, true);
        
        if(data.has_value()){
            Position_data.object_ROI_data.push_back(data.value()[0]);
        }
        else{
            Position_data.object_ROI_data.push_back(std::vector{cv::Point(-1, -1)});
        }
    }

    int target_index = -1;
    cv::Mat frame_HSV;
    for (int i = 0; i < Position_data.object_ROI_data.size(); i++){
        if(Position_data.object_ROI_data[i][0] == cv::Point(-1, -1)){
            continue;
        }
        target_index = i;
        Position_data.target_index = i;
        break;
    }

    int Point_num = lines;
    if (target_index != -1){
        if(lines == 0){//圆的情况，只有两个点，直接取第一个点,因为第二个点是(Radius, 0)
            Point_num = 1;
            Position_data.target_absolute_position.push_back(cv::Point((Position_data.object_ROI_data[Position_data.target_index][0].x ) + Position_data.roi_vector[Position_data.target_index].x, (Position_data.object_ROI_data[Position_data.target_index][0].y ) + Position_data.roi_vector[Position_data.target_index].y));
        }
        else{
            for (int i =0;i<Position_data.object_ROI_data[Position_data.target_index].size();i++){ 
                if(Position_data.object_ROI_data[Position_data.target_index][i] == cv::Point(-1, -1)){
                    Point_num--;
                    continue;
                }
                Position_data.target_absolute_position.push_back(cv::Point((Position_data.object_ROI_data[Position_data.target_index][i].x ) + Position_data.roi_vector[Position_data.target_index].x, (Position_data.object_ROI_data[Position_data.target_index][i].y ) + Position_data.roi_vector[Position_data.target_index].y));
            }
        }
    }
    
    return std::make_optional(Position_data);

}

/// @brief 将目标物体坐标映射到到画布坐标
/// @param frame_BGR 
/// @param frame_binary 
/// @return 应该画在画布上的绝对坐标
std::optional<ROI_with_oringin> find_object_positon_on_canvas(cv::Mat& frame_BGR, cv::Mat& frame_binary){
    std::vector<cv::Point> object_Relate_to_Contours_vectors;
    std::vector<double> object_Relate_to_Contours_cos_vector;
    std::vector<double> object_Relate_to_Contours_sin_vector;
    std::vector<cv::Point> object_vectors_on_canvas;
    std::optional<ROI_with_oringin> data;
    std::vector<int> lines = {0, 3, 4};

    for (const auto& i : lines){
        data = find_target_object(frame_BGR, frame_binary, i, 2);
        if (data.has_value()){
            if (data->target_index != -1){
                break;
            }
        }
    }

    for (int i = 0; i < data->target_absolute_position.size(); i++){
        if (data->target_absolute_position[i] != cv::Point(-1, -1)){
            object_Relate_to_Contours_vectors.push_back(data->target_absolute_position[i] - data->Contours_vertex[data->target_index][0]);
        }
    }

    double Ktc1 = 1;
    double Ktc2 = 1;
    double Ktc = 1;
    int canvas_index = 0;
    for (int i = 0; i < object_Relate_to_Contours_vectors.size(); i++){
        if (i == data->target_index){
            continue;
        }
        double weidth_target = sqrt(pow(data->Contours_vertex[data->target_index][0].x - data->Contours_vertex[data->target_index][3].x, 2) + pow(data->Contours_vertex[data->target_index][0].y - data->Contours_vertex[data->target_index][3].y, 2));
        double weidth_object = sqrt(pow(data->Contours_vertex[i][0].x - data->Contours_vertex[i][3].x, 2) + pow(data->Contours_vertex[i][0].y - data->Contours_vertex[i][3].y, 2));
        canvas_index = i;
        data->canvas_index = i;
        Ktc1 = weidth_object / weidth_target;

        weidth_target = sqrt(pow(data->Contours_vertex[data->target_index][1].x - data->Contours_vertex[data->target_index][2].x, 2) + pow(data->Contours_vertex[data->target_index][1].y - data->Contours_vertex[data->target_index][2].y, 2));
        weidth_object = sqrt(pow(data->Contours_vertex[i][1].x - data->Contours_vertex[i][2].x, 2) + pow(data->Contours_vertex[i][1].y - data->Contours_vertex[i][2].y, 2));
        Ktc2 = weidth_object / weidth_target;

        Ktc = (Ktc1 + Ktc2) / 2;
        break;
    }

    // std::cout << "Ktc:" << Ktc << " " << "Ktc1:" << Ktc1 << " " << "Ktc2:" << Ktc2 << std::endl;

    for(int i = 0;i<object_Relate_to_Contours_vectors.size();i++){
        double length = sqrt(pow(object_Relate_to_Contours_vectors[i].x, 2) + pow(object_Relate_to_Contours_vectors[i].y, 2));
        double cos = object_Relate_to_Contours_vectors[i].x / length;
        double sin = object_Relate_to_Contours_vectors[i].y / length;
        // std::cout << "length: " << length << "(" << object_Relate_to_Contours_vectors[i].x << ", " << object_Relate_to_Contours_vectors[i].y << ") cos: " << cos << " sin: " << sin << std::endl;
        object_Relate_to_Contours_vectors[i].y = static_cast<int>(length * Ktc * Ktc * sin);
        object_Relate_to_Contours_vectors[i].x = static_cast<int>(length * Ktc * Ktc * cos);
    }

    for (const auto& point : object_Relate_to_Contours_vectors){
        object_vectors_on_canvas.push_back(point + data->Contours_vertex[canvas_index][0]);
    }
    
    if (canvas_index != -1 && !object_vectors_on_canvas.empty()){
        data->object_vectors_on_canvas = object_vectors_on_canvas;
        

        //======================调试=========================
        // for(const auto& point : data->object_vectors_on_canvas){
        //     std::cout << "(" << point.x << ", " << point.y << ") ";
        // }
        // std::cout << std::endl;
        // // data->object_vectors_on_canvas[2].x = data->Contours_vertex[canvas_index][1].x;
        // if (data->object_vectors_on_canvas.size() == 4){

        //     data->object_vectors_on_canvas[0].y = std::max(data->object_vectors_on_canvas[0].y, data->object_vectors_on_canvas[3].y);
        //     if(data->object_vectors_on_canvas[3].y != data->Contours_vertex[canvas_index][0].y){
        //         data->object_vectors_on_canvas[3].y = data->Contours_vertex[canvas_index][0].y;
        //     }
            
        //     data->object_vectors_on_canvas[0].x = std::max(data->object_vectors_on_canvas[0].x, data->object_vectors_on_canvas[1].x);
        //     if(data->object_vectors_on_canvas[1].x != data->Contours_vertex[canvas_index][0].x){
        //         data->object_vectors_on_canvas[1].x = data->Contours_vertex[canvas_index][0].x;
        //     }

        //     data->object_vectors_on_canvas[2].y = data->object_vectors_on_canvas[1].y;

        //     data->object_vectors_on_canvas[2].x = std::min(data->object_vectors_on_canvas[3].x, data->object_vectors_on_canvas[2].x);
        //     if(data->object_vectors_on_canvas[3].x != data->Contours_vertex[canvas_index][2].x){
        //         data->object_vectors_on_canvas[3].x = data->Contours_vertex[canvas_index][2].x;
        //     }
        // }
        //======================调试=========================
        return data;
        
    }
    return std::nullopt;
}