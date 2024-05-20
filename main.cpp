#include <iostream>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

void ComPortHandler(const std::string& portName) {
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
    char buffer[256];
    while (true) {
        int n = read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            buffer[n] = '\0';
            std::cout << portName << " received: " << buffer << std::endl;
        }
        std::string dataToSend = "Hello from " + portName;
        write(fd, dataToSend.c_str(), dataToSend.size());
        usleep(1000000);  // Sleep for 1 second
    }

    close(fd);
}

int main() {
    std::vector<std::thread> threads;

    // Example ports, replace with actual port names on your system
    std::vector<std::string> portNames = {"/dev/ttyUSB0", "/dev/ttyUSB1"};

    for (const auto& portName : portNames) {
        threads.emplace_back(ComPortHandler, portName);
    }

    for (auto& t : threads) {
        t.join();
    }

    return 0;
}