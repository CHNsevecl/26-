#include <stdio.h>
#include "pico/stdlib.h"
#include "zdt.hpp"
#include "pico/stdio.h"
#include <unordered_map>
#include <optional>
#include <iostream>
#include <iomanip>
#include "Receive_uart.hpp"
#include <cstdlib>
#include <cmath>
#include "pico/multicore.h"
#include "PID.hpp"

#define addr2 0x02
#define addr1 0x01
#define acc_x 10
#define vel_x 255
#define acc_y 150
#define vel_y 150

std::string question_id = "0";

void print_debug(const std::string& s) {
    std::cout << "[";
    for (unsigned char c : s) {
        if (c >= 32 && c <= 126) {
            std::cout << c;
        } else {
            std::cout << "\\x" << std::hex << (int)c << std::dec;
        }
    }
    std::cout << "] ";
}

void core1_entry(){
    uart_init(uart1, 115200);
    gpio_set_function(8, GPIO_FUNC_UART);
    gpio_set_function(9, GPIO_FUNC_UART);
    sleep_ms(10);

    while (true){
        
        std::vector<std::string> datas = Receive();

        if(datas[0] != "RP5" || datas.back() != "END" || datas.size() < 4){
            continue;
        }

        question_id = datas[3];

        // for(int i = 0;i < datas.size();i++){
        //     print_debug(datas[i]);
        // }

        // std::cout << std::endl;

        if (!multicore_fifo_rvalid()){
            uint16_t data_x = std::stoi(datas[1]);
            uint16_t data_y = std::stoi(datas[2]);

            uint32_t package = ((uint32_t)data_x << 16) | data_y;
            multicore_fifo_push_blocking(package);
        }
        
    }

}

int main()
{
    stdio_init_all();

    // while(!stdio_usb_connected()){
    //     sleep_ms(500);
    // }

    ZDT_EmmV5 zdt;
    PIDController pid_x;

    pid_x.setGains(0.1,0.0,0.0001);
    pid_x.enableDebug(true);
    pid_x.setIntegralLimits(-5.0,5.0);
    pid_x.setOutputLimits(-255,255);
    PIDController pid_y;
    pid_y.setGains(0.3,0.0001,0.00001);
    pid_y.setIntegralLimits(-1.0,1.0);
    pid_y.setOutputLimits(-255,255);

    zdt.EmmV5_Init();
    int release_count = 0;

    multicore_launch_core1(core1_entry);

    // zdt.EmmV5_Pos_Control(addr1, 0x01, 100, 10, 11.2, 0x00, 0x00);
    // zdt.EmmV5_Vel_Control(addr1, 0x00, 1000, acc_x, 0x00);

    while (true){
        double mx;
        double my;
        // zdt.EmmV5_Vel_Control(addr1, 0x00, 20, acc_x, 0x00);
        int16_t dx = 0;
        int16_t dy = 0;

        if (multicore_fifo_rvalid()){
            int32_t package = multicore_fifo_pop_blocking();
            dx = (package >>16) & 0xFFFF;
            dy = (package & 0xFFFF);
        }
        else{
            continue;
        }

        
        if(question_id == "3"){
            if(std::abs(dx) > 200){
                pid_x.setKp(0.2);
            }
            else{
                pid_x.setKp(0.1);
            }
            if(std::abs(dy) > 100){
                pid_y.setKp(0.3);
            }
            else if(std::abs(dy) > 50){
                pid_y.setKp(0.2);
            }
            else{
                pid_y.setKp(0.1);
            }

            mx = pid_x.calculateAuto(dx,0.0);
            my = pid_y.calculateAuto(dy,0.0);

            if (std::abs(mx) < 1.0 && std::abs(mx) > 0.5) {
                mx = (mx < 0.0) ? -1.0 : 1.0;
            }
            if (std::abs(my) < 1.0 && std::abs(my) > 0.5) {
                my = (my < 0.0) ? -1.0 : 1.0;
            }

            std::cout << "dx:" << dx << " "<< "dy:" << dy<<std::endl;
            std::cout << "M:"<< mx <<" " << my <<std::endl;

            if (dx < 0){
                zdt.EmmV5_Pos_Control(addr1, 0x01, 10, acc_x, abs(dx), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr1, 0x01, (int)std::abs(mx), acc_x, 0x00);
            }
            else{
                zdt.EmmV5_Pos_Control(addr1, 0x00, 10, acc_x, abs(dx), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr1, 0x00, (int)mx, acc_x, 0x00);
            }
            
            if (dy < 0){
                zdt.EmmV5_Pos_Control(addr2, 0x00, 10, acc_x, abs(dy), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr2, 0x00, (int)std::abs(my), acc_x, 0x00);
            }
            else{
                zdt.EmmV5_Pos_Control(addr2, 0x01, 10, acc_x, abs(dy), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr2, 0x01, (int)std::abs(my), acc_x, 0x00);
            }
            
        }
        else if(question_id == "1" ){
            pid_x.setKp(0.3);
            pid_y.setKp(0.4);
            mx = pid_x.calculateAuto(dx,0.0);
            my = pid_y.calculateAuto(dy,0.0);
            std::cout << "dx:" << dx << " "<< "dy:" << dy<<std::endl;
            std::cout << "M:"<< mx <<" " << my <<std::endl;

            if (dx < 0){
                zdt.EmmV5_Pos_Control(addr1, 0x01, 10, acc_x, abs(dx), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr1, 0x01, (int)std::abs(mx), acc_x, 0x00);
            }
            else{
                zdt.EmmV5_Pos_Control(addr1, 0x00, 10, acc_x, abs(dx), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr1, 0x00, (int)mx, acc_x, 0x00);
            }
            
            if (dy < 0){
                zdt.EmmV5_Pos_Control(addr2, 0x00, 10, acc_x, abs(dy), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr2, 0x00, (int)std::abs(my), acc_x, 0x00);
            }
            else{
                zdt.EmmV5_Pos_Control(addr2, 0x01, 10, acc_x, abs(dy), 0x00, 0x00);
                // zdt.EmmV5_Vel_Control(addr2, 0x01, (int)std::abs(my), acc_x, 0x00);
            }
            
        }
        else if(question_id == "2" || question_id == "20"){
            double abs_dx = std::abs((double)dx);
            double abs_dy = std::abs((double)dy);
            double max_axis = (abs_dx > abs_dy) ? abs_dx : abs_dy;

            if (max_axis < 1.0) {
                continue;
            }

            int move_x = (int)std::round(abs_dx *2.0);
            int move_y = (int)std::round(abs_dy);

            

            std::cout << "Q2 dx:" << dx << " dy:" << dy << std::endl;
            std::cout << "Q2 px:" << move_x << " py:" << move_y << std::endl;

            if (dx < 0) {
                zdt.EmmV5_Pos_Control(addr1, 0x01, 10, acc_x, move_x, 0x00, 0x00);
            } else {
                zdt.EmmV5_Pos_Control(addr1, 0x00, 10, acc_x, move_x, 0x00, 0x00);
            }

            if (dy < 0) {
                zdt.EmmV5_Pos_Control(addr2, 0x00, 10, acc_x, move_y, 0x00, 0x00);
            } else {
                zdt.EmmV5_Pos_Control(addr2, 0x01, 10, acc_x, move_y, 0x00, 0x00);
            }
        }
        
        
            

        
        
    }
    
}