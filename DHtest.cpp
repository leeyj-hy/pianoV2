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
#include "whatTime.hpp"
#include "playThatSheet.hpp"

#define DEFAULTRPM 30


int _isM12Init = 0;
int _isM3Init = 0;
int _isM12PosSaved = 0;

std::string portLeftFingers = "/dev/ttyACM0";
std::string portRightFingers = "/dev/ttyACM1";
std::string portHandFoot = "/dev/ttyACM2";

std::string XMLLeftFingers = "../../Parameter/hand1param.xml";
std::string XMLRightFingers = "../../Parameter/hand2param.xml";
std::string XMLHandFoot = "../../Parameter/pedalparam.xml";

std::string XMLScore = "../../Score/score1.xml";

int main() {

    std::thread timeUpdater(updateCurrentTime);

    FingerMotor LfingerObj(portLeftFingers, L_FINGER, XMLLeftFingers);
    FingerMotor RfingerObj(portRightFingers, R_FINGER, XMLRightFingers);
    FingerMotor HandFootObj(portHandFoot, HAND, XMLHandFoot);


    char response = 0;
    std::vector<std::thread> threads;

    std::vector<std::string> portNames = {LfingerObj.g_acmPort, RfingerObj.g_acmPort, HandFootObj.g_acmPort};  // ttyACM

    

    while (!_isM12Init) {
        LfingerObj.InitializeAll();
        RfingerObj.InitializeAll();

        elapseFromLastCall();

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

    elapseFromLastCall();

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

    elapseFromLastCall();
    
    //elapseFromStart();

    std::cout << "Whole Module init done!" << std::endl;

    LfingerObj.startFeedback();
    RfingerObj.startFeedback();
    HandFootObj.startFeedback();
    setTimerStartPoint();
    while(1){
        response = 0;
        std::cout << "Press Test Start? [Y/n]" << std::endl;
        std::cin >> response;

        if (response == 'y' || response == 'Y') {
            std::cout << "Test Start!" << std::endl;

            for(int i=1; i<LfingerObj.g_deviceQuantity+1; i++){
                SCORE::pressKey(LfingerObj, i, DEFAULTRPM);
                usleep(1000000);
            }
            usleep(3000000);
            for(int i=1; i<LfingerObj.g_deviceQuantity+1; i++){
                SCORE::releaseKey(LfingerObj, i, DEFAULTRPM);
                usleep(1000000);
            }
            break;
        } else if (response == 'n' || response == 'N') {
            std::cout << "Test exit!" << std::endl;
        } else {
            std::cout << "No valid input! please retry!" << std::endl;
        }
    }
    

    for (auto& t : threads) {
        t.join();
    }

    while(1){

    }

    return 0;
}
