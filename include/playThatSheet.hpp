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
    /// @brief 키 번호와 rpm을 입력받아 해당 모터를 작동시키는 키 누르기 함수
    /// @param motor : 디바이스 객체
    /// @param key : 키 번호
    /// @param rpm  : 회전 속도
    void pressKey(FingerMotor &motor, uint8_t key, uint8_t rpm){
        if(key>motor.g_deviceQuantity || key<0){
            std::cerr << "Invalid Key" << std::endl;
            return;
        }
        
        uint8_t goalPosition = 0;
        motor.setPosition(key, rpm, goalPosition);
    }

    /// @brief 키 번호와 rpm을 입력받아 해당 모터를 작동시키는 키 떼기 함수
    /// @param motor : 디바이스 객체
    /// @param key : 키 번호
    /// @param rpm : 회전 속도
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
