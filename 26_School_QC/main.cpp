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
#define acc_x 175
#define vel_x 255
#define acc_y 150
#define vel_y 150

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

    pid_x.setGains(0.09,0.0001,0.0001);
    pid_x.setIntegralLimits(-1.0,0.0);
    pid_x.setOutputLimits(-255,255);
    PIDController pid_y;
    pid_y.setGains(0.09,0.0001,0.00001);
    pid_y.setIntegralLimits(-1.0,1.0);
    pid_y.setOutputLimits(-255,255);

    zdt.EmmV5_Init();
    int release_count = 0;

    multicore_launch_core1(core1_entry);

    // zdt.EmmV5_Pos_Control(addr1, 0x01, 100, 10, 11.2, 0x00, 0x00);
    while (true){
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

        

        int mx = pid_x.calculateAuto(dx,0.0);
        int my = pid_y.calculateAuto(dy,0.0);

        std::cout << "X: " << dx << " Y: " << dy << std::endl;
        std::cout << mx <<" " << my <<std::endl;

        if (std::abs(dx) <= 20 && std::abs(dx) > 1) {
            mx = 1;
        }
        if (std::abs(dy) <= 20 && std::abs(dy) > 1) {
            my = 1;
        }
        

        if (dx < 0){
            // zdt.EmmV5_Pos_Control(addr1, 0x00, mx, acc_x, abs(dx), 0x00, 0x00);
            zdt.EmmV5_Vel_Control(addr1, 0x00, abs(mx), acc_x, 0x00);
        }
        else{
            // zdt.EmmV5_Vel_Control(addr1, 0x01, vx, acc, 0x00);
            zdt.EmmV5_Vel_Control(addr1, 0x01, mx, acc_x, 0x00);
        }
        
            if (dy < 0){
                // zdt.EmmV5_Pos_Control(addr1, 0x00, mx, acc_x, abs(dx), 0x00, 0x00);
                zdt.EmmV5_Vel_Control(addr2, 0x00, abs(my), acc_x, 0x00);
            }
            else{
                // zdt.EmmV5_Vel_Control(addr1, 0x01, vx, acc, 0x00);
                zdt.EmmV5_Vel_Control(addr2, 0x01, abs(my), acc_x, 0x00);
            }
        
        
    }
    
}

