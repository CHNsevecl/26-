#include "stdint.h"
#include <stdio.h>
#include <vector>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "zdt.hpp"
#include <optional>
#include <unordered_map>    
#include <string>
#include <iostream>


#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1


/*! \brief 发送数据到舵机
*/
void ZDT_EmmV5::EmmV5_SendData(const std::vector<uint8_t>& data){
    uart_write_blocking(UART_ID, data.data(), data.size());
}

/*! \brief 初始化ZDT_EmmV5
* \return true表示初始化成功，false表示初始化失败
*/
bool ZDT_EmmV5::EmmV5_Init(){
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    sleep_ms(10);
    return true;
}

    

std::vector<uint8_t> ZDT_EmmV5::EmmV5_ReceiveData(uint8_t start,uint8_t end,size_t length, uint32_t timeout_ms){
    std::vector<uint8_t> data;
    data.reserve(length);
    bool start_flag = false;
    uint32_t timeout_us = timeout_ms * 1000;
    
    while (data.size() < length) {
        uint8_t byte;  // 先声明变量
        
        // 等待一个字节（最多timeout_us微秒）
        if (!uart_is_readable_within_us(UART_ID, timeout_us)) {
            break;  // 超时
        }
        
        byte = uart_getc(UART_ID);  // 只读取一次！
        
        if (!start_flag && byte == start) {
            start_flag = true;
            data.push_back(byte);
        }
        else if (start_flag) {
            data.push_back(byte);
            if (byte == end) {
                break;
            }
        }
    }
    
    return data;
}

/*! \brief  舵机位置控制
*  \details [addr, 0xFD, dir, vel_H, vel_L, acc, clk3, clk2, clk1, clk0, raF, snF, 0x6B]
*
* \param addr 舵机地址
* \param dir 舵机转动方向，0x00为CW，0x01为CCW
* \param vel 舵机转动速度，范围0x00-0xFF
* \param acc 舵机转动加速度，范围0x00-0xFF
* \param clk 脉冲数/步数，32 位无符号，高字节在前
* \param raF 相对/绝对标志位，0x00为相对位置控制，0x01为绝对位置控制
* \param snF 舵机同步标志位
*/
void ZDT_EmmV5::EmmV5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc,float angle, uint8_t raF, uint8_t snF){
    std::vector<uint8_t> command;
    uint32_t clk;
    clk = angle/11.2f *100 ;

    command.reserve(14);
    command.push_back(addr & 0xFF);
    command.push_back(0xFD);
    command.push_back(dir);
    command.push_back(vel >> 8);
    command.push_back(vel & 0xFF);
    command.push_back(acc);
    command.push_back(clk >> 24);
    command.push_back(clk >> 16);
    command.push_back(clk >> 8);
    command.push_back(clk & 0xFF);
    command.push_back(raF);
    command.push_back(snF);
    command.push_back(0x6B);
        
    // for (const auto& pair : command) {
    //         std::cout << pair ;
    //     }
    //     std::cout << std::endl;
    EmmV5_SendData(command);
    std::vector<uint8_t> response = EmmV5_ReceiveData(addr);

    // for (const auto& pair : response) {
    //         std::cout << pair ;
    //     }
    //     std::cout << std::endl;
        
    sleep_ms(10); // 等待舵机处理命令
}

/*! \brief  速度模式控制
 *  \details [addr, 0xF6, dir, vel_H, vel_L, acc, snF, 0x6B]
 *
 *  \param addr 舵机地址
 *  \param dir 舵机转动方向，0x00为CW，0x01为CCW
 *  \param vel 舵机转动速度，范围0x00-0xFF
 *  \param acc 舵机转动加速度，范围0x00-0xFF
 *  \param snF 舵机同步标志位
 */
void ZDT_EmmV5::EmmV5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF){
    std::vector<uint8_t> command;
    command.reserve(8);
    command.push_back(addr & 0xFF);
    command.push_back(0xF6);
    command.push_back(dir);
    command.push_back(vel >> 8);
    command.push_back(vel & 0xFF);
    command.push_back(acc);
    command.push_back(snF);
    command.push_back(0x6B);

    
    EmmV5_SendData(command);
    std::vector<uint8_t> response = EmmV5_ReceiveData(32,10,1);

    // for (const auto& pair : response) {
    //         std::cout << pair ;
    //     }
    //     std::cout << std::endl;

    sleep_ms(10); // 等待舵机处理命令
}


/*! \brief 读取系统参数。
* \details [addr, s, 0x6B]
* 
* \param s: 对应 C 枚举 SysParams_t 的编号。
* \param S_Conf(14) 需要附加 0x6C，S_State(15) 需要附加 0x7A。
* 末尾统一追加校验字节 0x6B。
*/
std::optional<std::unordered_map<std::string, float>> ZDT_EmmV5::EmmV5_Read_Sys_Params(uint8_t addr, uint8_t s, uint32_t timeout_ms){
    std::vector<uint8_t> command;
    command.reserve(3);
    command.push_back(addr & 0xFF);

    if (s == 0) {
        command.push_back(0x1F);
    }
    else if (s == 1) {
        command.push_back(0x20);
    }
    else if (s == 2) {
        command.push_back(0x21);
    }
    else if (s == 3) {
        command.push_back(0x24);
    }
    else if (s == 10) {
        command.push_back(0x36);
    }
    else {
        return std::nullopt;
    }
  
    command.push_back(0x6B);

    // for(const auto& n:command){
    //     std::cout << n;
    // }
    // std::cout << std::endl;

    EmmV5_SendData(command);

    sleep_ms(100);

    std::vector<uint8_t> response;
    response = EmmV5_ReceiveData(addr);

    // for(const auto& n:response){
    //     std::cout << n;
    // }
    // std::cout << std::endl;

    std::unordered_map<std::string, float> dic;
    if (s == 0) {
        dic["固件版本"] = ((response[5]<<8) | response[6]) /100.0f;
    }
    else if (s == 1) {
        dic["相电阻"] = response[5] / 100.0f;
        dic["相电感"] = response[6] / 100.0f;
    }
    else if (s == 2) {
        dic["PID_P"] = response[4] / 100.0f;
        dic["PID_I"] = response[5] / 100.0f;
        dic["PID_D"] = response[6] / 100.0f;
    }
    else if (s == 3) {
        dic["VBUS"] = ((response[5]<<8) | response[6]) /1000.0f;
    }
    else if (s == 10) {
        // for (const auto& pair : response) {
        //     std::cout << pair ;
        // }
        // std::cout << std::endl;
        dic["实时角度"] = (((response[3]<<24) | response[4]<<16 | response[5]<<8 | response[6]) / 182.04f);
        if (response[2] == 0x01){
            dic["实时角度"] = -dic["实时角度"];
        }
    }
    else {
        return std::nullopt;
    }
    
    return dic;
}