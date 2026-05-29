cmake_minimum_required(VERSION 3.10)
project(激光打靶)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)


find_package(OpenCV REQUIRED)

add_executable(hit 
            main.cpp 
            src/ImagePg.cpp
            src/uart.cpp
            src/Laser_Position.cpp
            QA/Answer.cpp)

target_link_libraries(hit ${OpenCV_LIBS})