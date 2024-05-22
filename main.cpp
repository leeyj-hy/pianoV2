#include <iostream>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <functional>
#include "Modbus.h"  // Modbus.h 헤더 파일 포함

// 콜백 함수 타입 정의
typedef std::function<void(const std::string&, const std::vector<uint8_t>&)> ReadCallback;

void ComPortHandler(const std::string& portName, ReadCallback callback) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << ": " << strerror(errno) << std::endl;
        return;
    }

    // Configure the port
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting termios attributes for " << portName << ": " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes for " << portName << ": " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    // Communication
    std::vector<uint8_t> buffer(6);  // 4 bytes data + 2 bytes CRC
    while (true) {
        int n = read(fd, buffer.data(), buffer.size());
        if (n > 0) {
            buffer.resize(n);
            callback(portName, buffer);
        }
    }

    close(fd);
}

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
    std::cout << "Sent to " << portName << ": ";
    for (uint8_t byte : message) {
        std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

void ReadCallbackFunction(const std::string& portName, const std::vector<uint8_t>& data) {
    /*std::cout << portName << " received: ";
    for (uint8_t byte : data) {
        std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
    */
}

int main() {
    std::vector<std::thread> threads;

    // Example ports, replace with actual port names on your system
    std::vector<std::string> portNames = {"/dev/ttyACM0", "/dev/ttyACM1"};

    for (const auto& portName : portNames) {
        threads.emplace_back(ComPortHandler, portName, ReadCallbackFunction);
    }

    // Example of writing a 6-byte message to ports (4 bytes data + 2 bytes CRC)
    std::vector<uint8_t> message = {0x01, 0x01, 0xff, 0x05};  // 임의의 데이터
    for (int i = 1; i < 16; i++) {
      for (const auto& portName : portNames) {
          message[1] = i;
          if(portName=="/dev/ttyACM0")
            message[0] = 1;
          else if(portName=="/dev/ttyACM1")
            message[0] = 2;
          WriteMessage(portName, message);
        }
        usleep(200000);
    }

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}
