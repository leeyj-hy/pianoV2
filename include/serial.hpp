#ifndef SERIAL_H
#define SERIAL_H


#include <iostream>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <functional>
#include <tinyxml2.h>
#include "Modbus.h"  // Modbus.h 헤더 파일 포함


#define SETHOME 1
#define SETZEROPOINT 2
#define SETPOSITION 3

struct _packet{
  uint8_t ID;
  uint8_t CMD;
  uint8_t Data1;
  uint8_t Data2;
};

// 콜백 함수 타입 정의
typedef std::function<void(const std::string&, const std::vector<uint8_t>&)> ReadCallback;

/// @brief 시리얼 포트를 열고 설정을 초기화하는 함수
/// @param portName 
/// @param message 
void WriteMessage(const std::string& portName, std::vector<uint8_t> message) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << " for writing: " << strerror(errno) << std::endl;
        return;
    }

    if (message.size() >= 4) {
        // 상위 4바이트에 대해 CRC 계산
        uint16_t crc = crc_modbus(message.data(), 4);
        // CRC 결과를 메시지의 마지막 2바이트에 추가
        message.push_back(crc & 0xFF);        // CRC LSB
        message.push_back((crc >> 8) & 0xFF); // CRC MSB
    } else {
        std::cerr << "Message length is less than 4 bytes." << std::endl;
        close(fd);
        return;
    }

    write(fd, message.data(), message.size());
    close(fd);

    // 터미널에 전송된 메시지를 출력
    /*std::cout << "Sent to " << portName << ": ";
    for (uint8_t byte : message) {
        std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;*/
}



void ReadCallbackFunction(const std::string& portName, const std::vector<uint8_t>& data) {
    /*std::cout << portName << " received: ";
    for (uint8_t byte : data) {
        std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
    */
}


#endif // SERIAL_H