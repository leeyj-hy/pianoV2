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




int _isM12Init=0;
int _isM3Init = 0;
int _isM12PosSaved = 0;

std::string portLeftFingers = "/dev/ttyACM0";
std::string portRightFingers = "/dev/ttyACM1";
std::string portHandFoot = "/dev/ttyACM2";




int main() {

  FingerMotor LfingerObj(portLeftFingers, L_FINGER);
  FingerMotor RfingerObj(portRightFingers, R_FINGER);
  FingerMotor HandFootObj(portHandFoot, HAND);

  char response = 0;
    std::vector<std::thread> threads;

    // Example ports, replace with actual port names on your system
    std::vector<std::string> portNames = {LfingerObj.g_acmPort, RfingerObj.g_acmPort, HandFootObj.g_acmPort};  //ttyACM

    // for (const auto& portName : portNames) {
    //     threads.emplace_back(ComPortHandler, portName, ReadCallbackFunction);
    // }

    // Read motor parameters from XML file
    // std::string paramFilePath = "/home/yj/piano2/Parameter/hand1param.xml";
    // std::cout << "Loading parameters from: " << paramFilePath << std::endl;
    // std::vector<MotorParameter> motorParameters = ReadMotorParametersFromXML(paramFilePath);

    // if (motorParameters.empty()) {
    //     std::cerr << "No motor parameters found. Exiting." << std::endl;
    //     return 1;
    // }
    // for(const auto& param : motorParameters){
    //   InitializeMotor(param);
    // }
    
    // Example of writing a 6-byte message to ports (4 bytes data + 2 bytes CRC)
    
    while(!_isM12Init){
      std::vector<uint8_t> message = {0x01, 0x01, 0xff, 0x05};  // 임의의 데이터
      for (int i = 1; i < 16; i++) {
        for (const auto& portName : portNames) {
            message[1] = i;
            if(portName==portLeftFingers)
              message[0] = 1;
            else if(portName==portRightFingers)
              message[0] = 2;
            WriteMessage(portName, message);
          }
          usleep(200000);
      }
      
      std::cout << "Module 1, 2 init End"<<std::endl;
      std::cout << "Progress to Module 3 Init sequence? [y/n]"<<std::endl;
      
      std::cin>>response;

      if(response =='y' | response == 'Y')
      {
        std::cout << "Module 1, 2 init done!"<<std::endl;
        
        _isM12Init=1;
        break;
      }
      else if(response == 'n' | response == 'N')
      {
        std::cout << "Module 1, 2 init Restart!"<<std::endl;
        
      }
      else{
        std::cout << "No valid input! please retry!"<<std::endl;
      }
    }
    
    while(!_isM3Init)
    {
      std::vector<uint8_t> message={0x03, 0x30, 0x10, 0x10};
      response = 0;
      std::cout << "Module 3 init is going to start!"<<std::endl;
      std::cout << "Remove every finger stuck?? [Y/n]"<<std::endl;

      std::cin>>response;

      if(response =='y' | response == 'Y')
      {
        std::cout << "Module 3 init start.."<<std::endl;
        usleep(1000000);  //10sec
        response = 0;
        std::cout << "Module 3 init success? [Y/n]"<<std::endl;
        std::cin>>response;

        if(response =='y' | response == 'Y')
      {
        
        
        _isM3Init=1;
        break;
      }
      else if(response == 'n' | response == 'N')
      {
        std::cout << "Module 3 init Restart!"<<std::endl;
        
      }
      else{
        std::cout << "No valid input! please retry!"<<std::endl;
      }
      }

      else{
        std::cout << "No valid input! please retry!"<<std::endl;
        
      }


      const auto& portName = portHandFoot;
      WriteMessage(portName, message);
    }

    std::cout << "Module init done!"<<std::endl;

    std::cout << "Modify Module 1, 2 init Pos? [Y/n]"<<std::endl;

    if(response =='y' | response == 'Y')
      {
        while(!_isM12PosSaved){
          std::cout << "==Select Module=="<<std::endl;
          std::cout << "l : Left fingers"<<std::endl;
          std::cout << "r : Right fingers"<<std::endl;
          std::cout << "n : Exit "<<std::endl;
          response=0;
          std::cin>>response;
          if(response == 'l')
          {
            
          }
          else if(response == 'r')
          {
            
          }
          else if(response == 'n')
          {

          }
          else
          {
            std::cout << "No valid input! please retry!"<<std::endl;
          }
        }

      }
    
    else{
        std::cout << "No valid input! please retry!"<<std::endl;
        while(1){}
      }

    for (auto& t : threads) {
        t.join();
    }

    while(1){

    }

    return 0;
}
