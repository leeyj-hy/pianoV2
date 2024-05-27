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
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>
#include <tinyxml2.h>
#include "Modbus.h"  // Modbus.h 헤더 파일 포함
#include "serial.hpp"

#define L_FINGER  0x01
#define R_FINGER  0x02
#define HAND      0x03

using namespace tinyxml2;
using namespace std;

struct MotorParameter {
    int id;
    int homePosition;
    int plusDir;
};

class FingerMotor
{
private:
    std::thread comPortThread;
    std::ofstream logfile;

    void ComPortHandler(const std::string& portName, std::function<void(const std::string&, const std::vector<uint8_t>&)> callback);
    static void ReadCallbackFunction(const std::string& portName, const std::vector<uint8_t>& data, std::ofstream& logfile, bool& prevWasFF);

public:
    int g_deviceQuantity = 0;
    std::string g_acmPort;
    std::vector<MotorParameter> motorParameters;
    std::vector<uint8_t> g_message = {0x00, 0x00, 0x00, 0x00};
    bool prevWasFF = false;

    FingerMotor(const std::string& acmPort, int deviceType, const std::string& _parameterPath);
    ~FingerMotor();
    std::string paramFilePath;
    uint8_t g_deviceType = 0;

    void LoadParameter();
    MotorParameter ReadMotorParameterByID(const std::string& filename, int motorID);
    void InitializeMotor(uint8_t motorID);
    void InitializeAll();
    void SaveHomePositionToXML(const std::string& filename, int motorID, int homePosition);
    void setPosition(uint8_t m_ID, uint8_t m_RPM, uint8_t m_GoalPosDegree);
    void setZeroPoint();
    void startFeedback();
};

FingerMotor::FingerMotor(const std::string& acmPort, int deviceType, const std::string& _parameterPath)
    : g_acmPort(acmPort), paramFilePath(_parameterPath), comPortThread(&FingerMotor::ComPortHandler, this, acmPort, [this](const std::string& portName, const std::vector<uint8_t>& data) { ReadCallbackFunction(portName, data, logfile, prevWasFF); })
{
    if (deviceType == L_FINGER) {
        g_deviceQuantity = 14;
        g_deviceType = L_FINGER;
    } else if (deviceType == R_FINGER) {
        g_deviceQuantity = 15;
        g_deviceType = R_FINGER;
    } else if (deviceType == HAND) {
        g_deviceQuantity = 3;
        g_deviceType = HAND;
    } else {
        cerr << "device type error" << endl;
        while (1) {}
    }

    XMLDocument doc;
    if (doc.LoadFile(_parameterPath.c_str()) != XML_SUCCESS) {
        std::cerr << "Error loading XML file: " << _parameterPath << std::endl;
    } else {
        cout << "XML loading Success!" << endl;
    }
}

FingerMotor::~FingerMotor()
{
    if (comPortThread.joinable()) {
        comPortThread.join();
    }
}

MotorParameter FingerMotor::ReadMotorParameterByID(const std::string& filename, int motorID) {
    MotorParameter param = {-1, -1, -1}; // 초기화 값, 해당 ID가 없으면 반환됨
    XMLDocument doc;
    cout << "read start" << endl;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        std::cerr << "Error loading XML file: " << filename << std::endl;
        return param;
    }
    cout << "xml loaded" << endl;
    XMLElement* root = doc.FirstChildElement("Motors");
    if (!root) {
        std::cerr << "No <Motors> element found in XML file." << std::endl;
        return param;
    }
    cout << "found <Motors> as root" << endl;
    for (XMLElement* elem = root->FirstChildElement("Motor"); elem != nullptr; elem = elem->NextSiblingElement("Motor")) {
        int id;
        cout << "finding ids" << endl;
        if (elem->FirstChildElement("ID")->QueryIntText(&id) == XML_SUCCESS && id == motorID) {
            cout << "found id " << id << endl;
            if (elem->FirstChildElement("HomePosition")->QueryIntText(&param.homePosition) == XML_SUCCESS &&
                elem->FirstChildElement("plusDir")->QueryIntText(&param.plusDir) == XML_SUCCESS) {
                cout << id <<endl;
                param.id = id;
                return param; // 해당 ID를 찾았으므로 바로 반환
            } else {
                std::cerr << "Error reading motor parameter values for ID " << motorID << " from XML file." << std::endl;
                return param;
            }
        }
    }

    std::cerr << "Motor ID " << motorID << " not found in XML file." << std::endl;
    return param; // 해당 ID가 없으면 초기화된 param 반환
}

void FingerMotor::LoadParameter(){
    for (int id = 1; id <= g_deviceQuantity; ++id) {
        MotorParameter param = ReadMotorParameterByID(paramFilePath, id);
        if (param.id != -1) {
            motorParameters.push_back(param);
        } else {
            std::cerr << "Failed to read motor parameters for ID " << id << std::endl;
        }
    }
    
    for (const auto& param : motorParameters) {
        std::cout << "Motor ID: " << param.id << ", Home Position: " << param.homePosition << ", Plus Direction: " << param.plusDir << std::endl;
    }
}

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

void FingerMotor::InitializeMotor(uint8_t motorID) {
    // LoadParameter();
}

void FingerMotor::InitializeAll() {
    LoadParameter();

    for (const auto& _id : motorParameters) {
        setPosition((uint8_t)_id.id, 0x10, (uint8_t)_id.homePosition);
        usleep(50000);
    }
}

void FingerMotor::setPosition(uint8_t m_ID, uint8_t m_RPM, uint8_t m_GoalPosDegree) {
    // 모터 위치 설정 로직 추가
    g_message = {0x00, m_ID, m_RPM, m_GoalPosDegree};
    if (FingerMotor::g_deviceType == L_FINGER) {
        g_message[0] = 0x01;
    } else if (FingerMotor::g_deviceType == R_FINGER) {
        g_message[0] = 0x02;
    } else if (FingerMotor::g_deviceType == HAND) {
        g_message[0] = 0x03;
    } else {
        std::cerr << "No valid Device Type" << std::endl;
        while (1) {}
    }
    
    if (m_ID > FingerMotor::g_deviceQuantity) {
        std::cerr << "No valid device ID" << std::endl;
        while (1) {}
    }

    if (m_RPM > 0xff || m_RPM < 0) {
        std::cerr << "No valid RPM input" << std::endl;
        while (1) {}
    }

    WriteMessage(g_acmPort, g_message);
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

    std::vector<uint8_t> buffer(256);
    while (true) {
        int n = read(fd, buffer.data(), buffer.size());
        if (n > 0) {
            buffer.resize(n);
            callback(portName, buffer);
        }
    }

    close(fd);
}

void FingerMotor::ReadCallbackFunction(const std::string& portName, const std::vector<uint8_t>& data, std::ofstream& logfile, bool& prevWasFF) {
    if (!logfile.is_open()) {
        std::cerr << "Log file is not open!" << std::endl;
        return;
    }
    for (uint8_t byte : data) {
        if (byte == 0xff) {
            if (prevWasFF) {
                logfile << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << std::endl;
                prevWasFF = false;
            } else {
                prevWasFF = true;
                logfile << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
            }
        } else {
            prevWasFF = false;
                logfile << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
    }
    
}

void FingerMotor::startFeedback(){
    g_message = {0x00, 0x30, 0x20, 0x20};
    std::string deviceSuffix;
    if (FingerMotor::g_deviceType == L_FINGER) {
        g_message[0] = 0x01;
        deviceSuffix = "_1";
    } else if (FingerMotor::g_deviceType == R_FINGER) {
        g_message[0] = 0x02;
        deviceSuffix = "_2";
    } else if (FingerMotor::g_deviceType == HAND) {
        g_message[0] = 0x03;
        deviceSuffix = "_3";
    } else {
        std::cerr << "No valid Device Type" << std::endl;
        while (1) {}
    }

    WriteMessage(g_acmPort, g_message);

    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S") << deviceSuffix;
    std::string filename = "../../Log/" + ss.str() + ".txt";

    // Log 디렉토리가 존재하는지 확인하고, 없으면 생성
    struct stat info;
    if (stat("../../Log", &info) != 0) {
        // Log 디렉토리가 존재하지 않음, 생성 시도
        if (mkdir("../../Log", 0777) == -1) {
            std::cerr << "Error creating Log directory: " << strerror(errno) << std::endl;
            return;
        }
    } else if (info.st_mode & S_IFDIR) {
        // Log 디렉토리가 이미 존재
    } else {
        std::cerr << "../../Log is not a directory!" << std::endl;
        return;
    }

    logfile.open(filename, std::ios::out);
    if (!logfile.is_open()) {
        std::cerr << "Error opening log file: " << filename << std::endl;
        return;
    }

    std::cout << "Log File " << filename << " created" << endl;
    std::cout << "Start Logging.." << endl;
}
#endif // FINGERMOTOR_H
