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
#include "FingerMotor.hpp"

int _isM12Init = 0;
int _isM3Init = 0;
int _isM12PosSaved = 0;

std::string portLeftFingers = "/dev/ttyACM0";
std::string portRightFingers = "/dev/ttyACM1";
std::string portHandFoot = "/dev/ttyACM2";

std::string XMLLeftFingers = "../../Parameter/hand1param.xml";
std::string XMLRightFingers = "../../Parameter/hand2param.xml";
std::string XMLHandFoot = "../../Parameter/pedalparam.xml";

int main() {
    FingerMotor LfingerObj(portLeftFingers, L_FINGER, XMLLeftFingers);
    FingerMotor RfingerObj(portRightFingers, R_FINGER, XMLRightFingers);
    FingerMotor HandFootObj(portHandFoot, HAND, XMLHandFoot);


    char response = 0;
    std::vector<std::thread> threads;

    std::vector<std::string> portNames = {LfingerObj.g_acmPort, RfingerObj.g_acmPort, HandFootObj.g_acmPort};  // ttyACM

    while (!_isM12Init) {
        LfingerObj.InitializeAll();
        RfingerObj.InitializeAll();

        std::cout << "Module 1, 2 init End" << std::endl;
        std::cout << "Progress to Module 3 Init sequence? [y/n]" << std::endl;

        std::cin >> response;

        if (response == 'y' || response == 'Y') {
            std::cout << "Module 1, 2 init done!" << std::endl;
            _isM12Init = 1;
            break;
        } else if (response == 'n' || response == 'N') {
            std::cout << "Module 1, 2 init Restart!" << std::endl;
        } else {
            std::cout << "No valid input! please retry!" << std::endl;
        }
    }

    while (!_isM3Init) {
        std::vector<uint8_t> message = {0x03, 0x30, 0x10, 0x10};
        response = 0;
        std::cout << "Module 3 init is going to start!" << std::endl;
        std::cout << "Remove every finger stuck?? [Y/n]" << std::endl;

        std::cin >> response;

        if (response == 'y' || response == 'Y') {
            std::cout << "Module 3 init start.." << std::endl;
            const auto& portName = portHandFoot;
            WriteMessage(portName, message);
            usleep(5000000);  // 5sec
            response = 0;
            std::cout << "Module 3 init success? [Y/n]" << std::endl;
            std::cin >> response;

            if (response == 'y' || response == 'Y') {
                _isM3Init = 1;
                break;
            } else if (response == 'n' || response == 'N') {
                std::cout << "Module 3 init Restart!" << std::endl;
            } else {
                std::cout << "No valid input! please retry!" << std::endl;
            }
        } else {
            std::cout << "No valid input! please retry!" << std::endl;
        }
    }

    std::cout << "Module init done!" << std::endl;
    std::cout << "Modify Module 1, 2 init Pos? [Y/n]" << std::endl;

    response = 0;
    std::cin >> response;

    if (response == 'y' || response == 'Y') {
        while (!_isM12PosSaved) {
            std::vector<uint8_t> message = {0x00, 0x00, 0x00, 0x00};
            uint8_t initpos = 0;
            std::cout << "==Select Module==" << std::endl;
            std::cout << "l : Left fingers" << std::endl;
            std::cout << "r : Right fingers" << std::endl;
            std::cout << "p : Pedal & Hand" << std::endl;
            std::cout << "n : Exit " << std::endl;

            response = 0;
            std::cin >> response;

            if (response == 'l') {
                std::cout << "Left Fingers selected" << std::endl;
                std::cout << "Select Motor ID num (1~14)" << std::endl;
                int _idInput = 0;
                std::cin >> _idInput;
                int _endZero = 0;
                int _degree = 0;
                if (_idInput > 0 && _idInput < 15) {
                    std::cout << _idInput << " Selected" << std::endl;
                    while (!_endZero) {
                        response = 0;
                        std::cout << "command input" << std::endl;
                        std::cout << "finger up(+1 degree) : u" << std::endl;
                        std::cout << "finger dn(-1 degree) : d" << std::endl;
                        std::cout << "Save Pos and Leave : s" << std::endl;
                        std::cout << "Leave without Save : l" << std::endl;

                        std::cin >> response;
                        if (response == 'u') {
                            _degree += 1;
                            LfingerObj.setPosition(_idInput, 0xff, _degree);
                            std::cout << "Current Position of " << _idInput << ": " << _degree << std::endl;
                        } else if (response == 'd') {
                            _degree -= 1;
                            LfingerObj.setPosition(_idInput, 0xff, _degree);
                            std::cout << "Current Position of " << _idInput << ": " << _degree << std::endl;
                        } else if (response == 's') {
                            std::cout << "Save Current Position and Leave" << std::endl;
                            std::string paramFilePath = "../../Parameter/hand1param.xml";
                            LfingerObj.SaveHomePositionToXML(paramFilePath, _idInput, _degree);
                            _endZero = 1;
                        } else if (response == 'l') {
                            std::cout << "Leave without Save" << std::endl;
                            _endZero = 1;
                        } else {
                            std::cout << "No valid input! please retry!" << std::endl;
                        }
                    }
                }
            } else if (response == 'r') {
                std::cout << "Right Fingers selected" << std::endl;
                std::cout << "Select Motor ID num (1~14)" << std::endl;
                int _idInput = 0;
                std::cin >> _idInput;
                int _endZero = 0;
                int _degree = 0;
                if (_idInput > 0 && _idInput < 14) {
                    std::cout << _idInput << " Selected" << std::endl;
                    while (!_endZero) {
                        response = 0;
                        std::cout << "command input" << std::endl;
                        std::cout << "finger up(+1 degree) : u" << std::endl;
                        std::cout << "finger dn(-1 degree) : d" << std::endl;
                        std::cout << "Save Pos and Leave : s" << std::endl;
                        std::cout << "Leave without Save : l" << std::endl;

                        std::cin >> response;
                        if (response == 'u') {
                            _degree += 1;
                            RfingerObj.setPosition(_idInput, 0xff, _degree);
                            std::cout << "Current Position of " << _idInput << ": " << _degree << std::endl;
                        } else if (response == 'd') {
                            _degree -= 1;
                            RfingerObj.setPosition(_idInput, 0xff, _degree);
                            std::cout << "Current Position of " << _idInput << ": " << _degree << std::endl;
                        } else if (response == 's') {
                            std::cout << "Save Current Position and Leave" << std::endl;
                            std::string paramFilePath = "../../Parameter/hand1param.xml";
                            RfingerObj.SaveHomePositionToXML(paramFilePath, _idInput, _degree);
                            _endZero = 1;
                        } else if (response == 'l') {
                            std::cout << "Leave without Save" << std::endl;
                            _endZero = 1;
                        } else {
                            std::cout << "No valid input! please retry!" << std::endl;
                        }
                    }
                }
            } else if (response == 'h') {
                std::cout << "Pedal & Hand selected" << std::endl;
                std::cout << "Select Motor ID num (1~3)" << std::endl;
            } else if (response == 'n') {
                _isM12PosSaved = 1;
            } else {
                std::cout << "No valid input! please retry!" << std::endl;
            }
        }
    } else {
        std::cout << "No valid input! please retry!" << std::endl;
        while (1) {}
    }

    for (auto& t : threads) {
        t.join();
    }

    while (1) {}

    return 0;
}
