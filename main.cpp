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
        "video/x-raw,format=NV12,width=640,height=480,framerate=15/1 ! "
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
    if (!uart.init("/dev/serial0", 115200)) {
        return -1;
    }

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
        cv::threshold(frame_gray, frame_binary, 90, 255, cv::THRESH_BINARY_INV);

        //===============形态学去噪=======================
        morphologyProcess(frame_binary, 3);


        //================ 流数据处理 =================

        


        //user_code_begin
        Question1_Answer(uart, frame_BGR, frame_binary);

        //user_code_end




        cv::imshow("Line",frame_BGR);
        if (cv::waitKey(1) == 27){
            break;
        }
    }


    uart.close_port();
    return 0;
}