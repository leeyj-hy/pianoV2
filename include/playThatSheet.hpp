#ifndef PLAYTHATSHEET_H
#define PLAYTHATSHEET_H

#include <iostream>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <functional>
#include <tinyxml2.h>

namespace SCORE{

    using namespace tinyxml2;
    using namespace std;

    int loadScoreFromXML(){
        int retVal = -1;
        
        return retVal;
    }

    void calKey(){

    }

    void pressKey(FingerMotor &motor, uint8_t key, uint8_t rpm){
        if(key>motor.g_deviceQuantity || key<0){
            std::cerr << "Invalid Key" << std::endl;
            return;
        }
        
        uint8_t goalPosition = 0;
        motor.setPosition(key, rpm, goalPosition);
    }

    void releaseKey(FingerMotor &motor, uint8_t key, uint8_t rpm){
        if(key>motor.g_deviceQuantity || key<0){
            std::cerr << "Invalid Key" << std::endl;
            return;
        }
        MotorParameter param = motor.ReadMotorParameterByID(motor.paramFilePath, key);
        uint8_t goalPosition = param.homePosition;

        motor.setPosition(key, rpm, goalPosition);
    }



}



#endif // !PLAYTHATSHEET_H
