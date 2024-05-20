#include <iostream>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <functional>

// 콜백 함수 타입 정의
typedef std::function<void(const std::string&, uint8_t)> ReadCallback;

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

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

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
    uint8_t buffer;
    while (true) {
        int n = read(fd, &buffer, 1);
        if (n > 0) {
            callback(portName, buffer);
        }
        usleep(100000);  // Sleep for 0.1 second
    }

    close(fd);
}

void WriteByte(const std::string& portName, uint8_t data) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << portName << " for writing: " << strerror(errno) << std::endl;
        return;
    }
    
    write(fd, &data, 1);
    close(fd);
}

void ReadCallbackFunction(const std::string& portName, uint8_t data) {
    std::cout << portName << " received: 0x" << std::hex << static_cast<int>(data) << std::dec << std::endl;
}

int main() {
    std::vector<std::thread> threads;

    // Example ports, replace with actual port names on your system
    std::vector<std::string> portNames = {"/dev/ttyUSB0", "/dev/ttyUSB1"};

    for (const auto& portName : portNames) {
        threads.emplace_back(ComPortHandler, portName, ReadCallbackFunction);
    }

    // Example of writing data to ports
    WriteByte("/dev/ttyUSB0", 0x41);  // Send 'A' (0x41 in hex)
    WriteByte("/dev/ttyUSB1", 0x42);  // Send 'B' (0x42 in hex)

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}
