#ifndef RECEIVE_UART_HPP
#define RECEIVE_UART_HPP

#include <vector>
#include <string>

std::vector<std::string> Receive();
void UART_SendData(const std::vector<std::string>& data);

#endif

