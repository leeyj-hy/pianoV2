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

using namespace tinyxml2;
// 콜백 함수 타입 정의
typedef std::function<void(const std::string&, const std::vector<uint8_t>&)> ReadCallback;

struct MotorParameter{
  int id;
  int homePosition;
  int plusDir;
};

int _isM12Init=0;
int _isM3Init = 0;

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

std::vector<MotorParameter> ReadMotorParametersFromXML(const std::string& filename) {
    std::vector<MotorParameter> parameters;
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        std::cerr << "Error loading XML file: " << filename << std::endl;
        return parameters;
    }

    XMLElement* root = doc.FirstChildElement("Motors");
    if (!root) {
        std::cerr << "No <Motors> element found in XML file." << std::endl;
        return parameters;
    }

    for (XMLElement* elem = root->FirstChildElement("Motor"); elem != nullptr; elem = elem->NextSiblingElement("Motor")) {
        MotorParameter param;
        if (elem->FirstChildElement("ID")->QueryIntText(&param.id) == XML_SUCCESS &&
            elem->FirstChildElement("HomePosition")->QueryIntText(&param.homePosition) == XML_SUCCESS &&
            elem->FirstChildElement("OtherParameter")->QueryIntText(&param.plusDir) == XML_SUCCESS) {
            parameters.push_back(param);
        } else {
            std::cerr << "Error reading motor parameter values from XML file." << std::endl;
        }
    }

    return parameters;
}

void InitializeMotor(const MotorParameter& param) {
    // 모터 초기화 로직 추가
    std::cout << "Initializing motor ID " << param.id << " to home position " << param.homePosition 
              << " with other parameter " << param.plusDir << std::endl;
    
    // // 실제 모터 초기화 명령을 시리얼 포트로 전송 (예시)
    // std::vector<uint8_t> message = {static_cast<uint8_t>(param.id), 
    //                                 static_cast<uint8_t>(param.homePosition & 0xFF), 
    //                                 static_cast<uint8_t>((param.homePosition >> 8) & 0xFF), 
    //                                 static_cast<uint8_t>(param.plusDir)};
    
    // // 예제 포트로 메시지를 전송
    // // 실제 포트 이름은 시스템에 따라 다를 수 있습니다
    // WriteMessage("/dev/ttyUSB0", message);
}

int main() {

  char response = 0;
    std::vector<std::thread> threads;

    // Example ports, replace with actual port names on your system
    std::vector<std::string> portNames = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"};  //ttyACM

    for (const auto& portName : portNames) {
        threads.emplace_back(ComPortHandler, portName, ReadCallbackFunction);
    }

    // Read motor parameters from XML file
    // std::string paramFilePath = "/home/yj/piano2/Parameter/hand1param.xml";
    // std::cout << "Loading parameters from: " << paramFilePath << std::endl;
    // std::vector<MotorParameter> motorParameters = ReadMotorParametersFromXML(paramFilePath);

    // if (motorParameters.empty()) {
    //     std::cerr << "No motor parameters found. Exiting." << std::endl;
    //     return 1;
    // }
    // for(const auto& param : motorParameters){
    //   InitializeMotor(param);
    // }
    
    // Example of writing a 6-byte message to ports (4 bytes data + 2 bytes CRC)
    
    while(!_isM12Init){
      std::vector<uint8_t> message = {0x01, 0x01, 0xff, 0x05};  // 임의의 데이터
      for (int i = 1; i < 16; i++) {
        for (const auto& portName : portNames) {
            message[1] = i;
            if(portName=="/dev/ttyUSB0")
              message[0] = 1;
            else if(portName=="/dev/ttyUSB1")
              message[0] = 2;
            WriteMessage(portName, message);
          }
          usleep(200000);
      }
      
      std::cout << "Module 1, 2 init End"<<std::endl;
      std::cout << "Progress to Module 3 Init sequence? [y/n]"<<std::endl;
      
      std::cin>>response;

      if(response =='y' | response == 'Y')
      {
        std::cout << "Module 1, 2 init done!"<<std::endl;
        
        _isM12Init=1;
        break;
      }
      else if(response == 'n' | response == 'N')
      {
        std::cout << "Module 1, 2 init Restart!"<<std::endl;
        
      }
      else{
        std::cout << "No valid input! please retry!"<<std::endl;
      }
    }
    
    while(!_isM3Init)
    {
      std::vector<uint8_t> message={0x03, 0x30, 0x10, 0x10};
      response = 0;
      std::cout << "Module 3 init is going to start!"<<std::endl;
      std::cout << "Remove every finger stuck??"<<std::endl;

      std::cin>>response;

      if(response =='y' | response == 'Y')
      {
        std::cout << "Module 3 init start"<<std::endl;
        
      }
      else{
        std::cout << "No valid input! please retry!"<<std::endl;
        while(1){

        }
      }
      const auto& portName = "/dev/ttyUSB2";
      WriteMessage(portName, message);
    }

    for (auto& t : threads) {
        t.join();
    }

    while(1){

    }

    return 0;
}
