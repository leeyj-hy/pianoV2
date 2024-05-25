#ifndef FINGERMOTOR_H
#define FINGERMOTOR_H

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
#include "serial.hpp"

#define L_FINGER  1
#define R_FINGER  2
#define HAND      3

using namespace tinyxml2;
using namespace std;

class FingerMotor
{
private:
    
    int g_deviceQuantity = 0;
    std::thread comPortThread;

    void ComPortHandler(const std::string& portName, std::function<void(const std::string&, const std::vector<uint8_t>&)> callback);
    static void ReadCallbackFunction(const std::string& portName, const std::vector<uint8_t>& data);

public:
    std::string g_acmPort = nullptr;

    std::vector<uint8_t> g_message = {0x00, 0x00, 0x00, 0x00};

    FingerMotor(std::string acmPort, int deviceType);
    ~FingerMotor();

    struct MotorParameter;
    // std::vector<MotorParameter> FingerMotor::ReadMotorParametersFromXML(const std::string& filename);
    void InitializeMotor(const MotorParameter& param);

    void SaveHomePositionToXML(const std::string& filename, int motorID, int homePosition);

    void setPosition(int m_ID, uint8_t m_RPM, uint8_t m_GoalPosDegree);
    void setZeroPoint();
    void startFeedback();
    
};

struct MotorParameter {
    int id;
    int homePosition;
    int plusDir;
};

FingerMotor::FingerMotor(std::string acmPort, int deviceType)
    : g_acmPort(acmPort), comPortThread(&FingerMotor::ComPortHandler, this, std::string(acmPort), ReadCallbackFunction)
{
    if (deviceType == L_FINGER) {
        g_deviceQuantity = 14;
    }
    else if (deviceType == R_FINGER) {
        g_deviceQuantity = 15;
    }
    else if (deviceType == HAND) {
        g_deviceQuantity = 3;
    }
    else {
        cout << "device type error" << endl;
        while (1) {}
    }
}

FingerMotor::~FingerMotor()
{
    if (comPortThread.joinable()) {
        comPortThread.join();
    }
}

// std::vector<MotorParameter> FingerMotor::ReadMotorParametersFromXML(const std::string& filename) {
//     std::vector<MotorParameter> parameters;
//     XMLDocument doc;
//     if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
//         std::cerr << "Error loading XML file: " << filename << std::endl;
//         return parameters;
//     }

//     XMLElement* root = doc.FirstChildElement("Motors");
//     if (!root) {
//         std::cerr << "No <Motors> element found in XML file." << std::endl;
//         return parameters;
//     }

//     for (XMLElement* elem = root->FirstChildElement("Motor"); elem != nullptr; elem = elem->NextSiblingElement("Motor")) {
//         MotorParameter param;
//         if (elem->FirstChildElement("ID")->QueryIntText(&param.id) == XML_SUCCESS &&
//             elem->FirstChildElement("HomePosition")->QueryIntText(&param.homePosition) == XML_SUCCESS &&
//             elem->FirstChildElement("OtherParameter")->QueryIntText(&param.plusDir) == XML_SUCCESS) {
//             parameters.push_back(param);
//         } else {
//             std::cerr << "Error reading motor parameter values from XML file." << std::endl;
//         }
//     }

//     return parameters;
// }

void FingerMotor::SaveHomePositionToXML(const std::string& filename, int motorID, int homePosition) {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        std::cerr << "Error loading XML file: " << filename << std::endl;
        return;
    }

    XMLElement* root = doc.FirstChildElement("Motors");
    if (!root) {
        std::cerr << "No <Motors> element found in XML file." << std::endl;
        return;
    }

    for (XMLElement* elem = root->FirstChildElement("Motor"); elem != nullptr; elem = elem->NextSiblingElement("Motor")) {
        int id;
        if (elem->FirstChildElement("ID")->QueryIntText(&id) == XML_SUCCESS && id == motorID) {
            XMLElement* homePosElem = elem->FirstChildElement("HomePosition");
            if (homePosElem) {
                homePosElem->SetText(homePosition);
                doc.SaveFile(filename.c_str());
                std::cout << "Home position for motor ID " << motorID << " saved to " << homePosition << std::endl;
                return;
            }
        }
    }

    std::cerr << "Motor ID " << motorID << " not found in XML file." << std::endl;
}

void FingerMotor::InitializeMotor(const MotorParameter& param) {
    
}

void FingerMotor::setPosition(int m_ID, uint8_t m_RPM, uint8_t m_GoalPosDegree) {
    // 모터 위치 설정 로직 추가
    
}



void FingerMotor::ComPortHandler(const std::string& portName, std::function<void(const std::string&, const std::vector<uint8_t>&)> callback) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << ": " << strerror(errno) << std::endl;
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting termios attributes for " << portName << ": " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting termios attributes for " << portName << ": " << strerror(errno) << std::endl;
        close(fd);
        return;
    }

    std::vector<uint8_t> buffer(6);
    while (true) {
        int n = read(fd, buffer.data(), buffer.size());
        if (n > 0) {
            buffer.resize(n);
            callback(portName, buffer);
        }
    }

    close(fd);
}

void FingerMotor::ReadCallbackFunction(const std::string& portName, const std::vector<uint8_t>& data) {
    std::cout << portName << " received: ";
    for (uint8_t byte : data) {
        std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
    }
    std::cout << std::dec << std::endl;
}

#endif // FINGERMOTOR_H
