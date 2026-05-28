#include <vector>
#include <string>
#include <cstdint>
#include "pico/stdio.h"
#include "pico/stdlib.h"

std::vector<std::string> Receive() {
    std::vector<std::string> datas;
    datas.reserve(5);
    
    std::string current_data;

    if (!uart_is_readable_within_us(uart1, 500000)) {
        return datas;  // 真正超时才返回空
    }

    
    while (true) {
        // 等待数据，带超时
        if (!uart_is_readable_within_us(uart1, 10000)) {  // 100ms超时
            break;  // 超时退出
        }
        
        char byte = uart_getc(uart1);
        
        // 遇到换行符，结束接收
        if (byte == '\n') {
            if (!current_data.empty()) {
                datas.push_back(current_data);
            }
            break;
        }
        
        // 遇到空格，表示一个字段结束
        if (byte == ' ') {  // 0x20
            if (!current_data.empty()) {
                datas.push_back(current_data);
                current_data.clear();
            }
        }
        else if (byte == '\r'){
            continue;
        }
        else {
            current_data.push_back(byte);
        }
    }
    
    return datas;
}

void UART_SendData(const std::vector<std::string>& data){
    for (const auto& s : data) {
        uart_write_blocking(uart1,reinterpret_cast<const uint8_t*>(s.data()),s.size());
        uart_write_blocking(uart1,reinterpret_cast<const uint8_t*>(" "),1);  // 字段之间加空格
        
    }
}