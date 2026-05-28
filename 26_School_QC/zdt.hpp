#ifndef ZDT_HPP
#define ZDT_HPP

#include <stdint.h>
#include <stddef.h>
#include <vector>
#include <optional>
#include <unordered_map>
#include <string>

class ZDT_EmmV5 {
protected:
    void EmmV5_SendData(const std::vector<uint8_t>& data);

public:
    ZDT_EmmV5() = default;

    bool EmmV5_Init();
    
    std::vector<uint8_t> EmmV5_ReceiveData(uint8_t start = 0x6B,uint8_t end = 0x6B,size_t length = 32, uint32_t timeout_ms = 10);
    
    void EmmV5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, float angle, uint8_t raF, uint8_t snF);

    void EmmV5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint8_t snF);

    std::optional<std::unordered_map<std::string, float>> EmmV5_Read_Sys_Params(uint8_t addr, uint8_t s, uint32_t timeout_ms=200);
};

#endif // ZDT_HPP