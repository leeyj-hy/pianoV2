#ifndef dBPort_hpp
#define dBPort_hpp

#include <iostream>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <mutex>
#include <condition_variable>

class dBPort {
public:
    dBPort(const std::string& portName);
    ~dBPort();
    bool sendByte(uint8_t byte);
    uint8_t receiveByte();
    int getResponse();


private:
    std::string portName;
    int fd;
    std::thread readThread;
    uint8_t receivedByte;
    bool receivedFlag;

    void readFunction();
};

dBPort::dBPort(const std::string& portName) : portName(portName), receivedFlag(false) {
    fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
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
    tty.c_cc[VMIN] = 1;
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

    readThread = std::thread(&dBPort::readFunction, this);
}

dBPort::~dBPort() {
    if (readThread.joinable()) {
        readThread.join();
    }
    close(fd);
}

bool dBPort::sendByte(uint8_t byte) {
    if(write(fd, &byte, 1)) return 1;
    else return 0;
}

uint8_t dBPort::receiveByte() {
    uint8_t byte;
    read(fd, &byte, 1);
    return byte;
}

int dBPort::getResponse() {
    while (!receivedFlag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    receivedFlag = false;
    return receivedByte;
}

void dBPort::readFunction() {
    while (true) {
        uint8_t byte = receiveByte();
        if (byte != 0) {
            receivedByte = byte;
            receivedFlag = true;
        }
    }
}

#endif // !define dBPort_hpp