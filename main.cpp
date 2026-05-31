#include <opencv2/opencv.hpp>
#include <iostream>
#include "src/uart.hpp"
#include <cstdlib>
#include <algorithm>
#include "src/ImagePg.hpp"
#include "src/Laser_Position.hpp"
#include "QA/Answer.hpp"


int main(){
    setenv("DISPLAY",":0",1);

    //================初始化摄像头================
    std::string pipeline = 
    {
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
        "video/x-raw,format=NV12,width=1850,height=620,framerate=15/1 ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    };

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if(!cap.isOpened()){
        std::cerr << "Error opening video stream" << std::endl;
        return -1;
    }
    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    //================================

    //============初始化UART============
    UART uart;
    UART uart2;
    if(!uart.init("/dev/serial0", 115200)){
        return -1;
    }
    if (!uart2.init("/dev/ttyAMA2", 115200)) {
        return -1;
    }

    int drawed_points = 0;
    bool finish_flag = false;
    std::optional<ROI_with_oringin> data4;
    std::optional<ROI_with_oringin> data3;
    //============处理视频流============
    while (true){
        //================ 流数据处理 =================


        cv::Mat frame_BGR;
        cv::Mat frame_gray;
        cv::Mat frame_binary;
        cap >> frame_BGR;

        if (frame_BGR.empty()){
            std::cerr << "Error reading frame" << std::endl;
            break;
        }
        cv::cvtColor(frame_BGR, frame_gray, cv::COLOR_BGR2GRAY);
        cv::threshold(frame_gray, frame_binary, 60, 255, cv::THRESH_BINARY_INV);

        //===============形态学去噪=======================
        morphologyProcess(frame_binary, 3);


        //================ 流数据处理 =================

        


        // Q4 - 画布移动 + 云台逐点画画（同Q2逻辑，画完回到第一个点后结束）
        int returned_num = Question4_Answer(uart, uart2, data4, frame_BGR, frame_binary, drawed_points, finish_flag);
        if(returned_num == -1){
            std::cout << "Q4 - Back to first point! Exiting." << std::endl;
            send_direction_to_MCU(uart, cv::Point(0, 0), 0, "RP5", "END");
            break;
        }
        else if(returned_num == 1){
            drawed_points++;
        }


        // Q1
        // if(Question1_Answer(uart, frame_BGR, frame_binary) == -1){
        //     break;
        // }

        // Q2
        // int returned_num = Question2_Answer(uart, frame_BGR, frame_binary, drawed_points);
        // if(returned_num == -1){
        //     std::cout << "All points have been drawn!" << std::endl;
        //     break;
        // }
        // else if(returned_num == 1){
        //     sleep(0.1);
        // }
        // drawed_points += returned_num;

        // //Q3
        // if(Question3_Answer(uart, uart2, data3, frame_BGR, frame_binary) == -1){
        //     break;
        // }

        // if(data.roi_vector.size() > 0) {
        //     cv::Mat frame_BGR_copy = frame_BGR.clone();
        //     frame_BGR_copy = frame_BGR_copy(data.roi_vector[0]);
        //     cv::imshow("Frame", frame_BGR_copy);
        // }

        // cv::imshow("Frame", frame_BGR.roi(data.roi_vector[0]));
        cv::imshow("Frame", frame_binary);
        cv::imshow("Line",frame_BGR);
        if (cv::waitKey(1) == 27){
            break;
        }
    }

    send_direction_to_MCU(uart, cv::Point(0, 0), 0, "RP5", "END");
    uart.close_port();
    uart2.close_port();
    return 0;
}