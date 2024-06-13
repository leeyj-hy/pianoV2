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
    dBPort(const std::string& portName) : portName(portName), fd(-1) {}
    bool openPort();
    void closePort();
    bool writeByte(uint8_t byte);

    int readByte();

    void notifyDataReceived(uint8_t byte);
    void waitForData();
    uint8_t getReceivedData() const;

private:
    std::string portName;
    int fd;
    std::mutex mtx;
    std::condition_variable cv;
    bool dataReceived;
    uint8_t receivedData;
};

bool dBPort::openPort() {
        fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            std::cerr << "Error opening " << portName << ": " << strerror(errno) << std::endl;
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0) {
            std::cerr << "Error getting termios attributes for " << portName << ": " << strerror(errno) << std::endl;
            close(fd);
            return false;
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
            return false;
        }

        return true;
}

void dBPort::closePort() {
    if (fd >= 0) {
        close(fd);
    }
}

bool dBPort::writeByte(uint8_t byte) {
        if (fd < 0) {
            std::cerr << "Port not open" << std::endl;
            return false;
        }
        int n = write(fd, &byte, 1);
        return n == 1;
}

int dBPort::readByte() {
        if (fd < 0) {
            //std::cerr << "Port not open" << std::endl;
            return -1;
        }
        uint8_t byte;
        int n = read(fd, &byte, 1);
        if (n == 1) {
            return byte;
        } else {
            return -1;
        }
    }


void dBPort::notifyDataReceived(uint8_t byte) {
    std::unique_lock<std::mutex> lock(mtx);
    receivedData = byte;
    dataReceived = true;
    cv.notify_one();
}

void dBPort::waitForData() {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [this]() { return dataReceived; });
    dataReceived = false;
}

uint8_t dBPort::getReceivedData() const {
    return receivedData;
}

void receiveBytes(dBPort& port) {
    while (true) {
        port.readByte();
    }
}

#endif // !define dBPort_hpp