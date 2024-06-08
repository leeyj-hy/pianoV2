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
    
    std::vector<SCORE::NOTE> notes;

    std::cout << SCORE::loadScoreFromXML(XMLScore, notes) <<  std::endl;

    

    std::thread timeUpdater(updateCurrentTime);

    FingerMotor LfingerObj(portLeftFingers, L_FINGER, XMLLeftFingers);
    FingerMotor RfingerObj(portRightFingers, R_FINGER, XMLRightFingers);
    FingerMotor HandFootObj(portHandFoot, HAND, XMLHandFoot);
    usleep(1000000);
    std::cout << "score loading success!" << std::endl;
    std::cout << "tempo : " << SCORE::Tempo << " time signature : " << SCORE::Beats << "/" << SCORE::BeatType << std::endl;
    
    SCORE::calKey(notes, HandFootObj);
    //while(1){}

    
    char response = 0;
    std::vector<std::thread> threads;

    std::vector<std::string> portNames = {LfingerObj.g_acmPort, RfingerObj.g_acmPort, HandFootObj.g_acmPort};  // ttyACM

    //setTimerStartPoint();

    while (!_isM12Init) {
        LfingerObj.InitializeAll();
        RfingerObj.InitializeAll();

        //elapseFromLastCall();
        //elapseFromStart();

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
    HandFootObj.LoadParameter();
    //elapseFromLastCall();
    //elapseFromStart();

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
                HandFootObj.setPosition(HandFootObj.motorParameters[0].id, 0x5, HandFootObj.motorParameters[0].homePosition);
                HandFootObj.setPosition(HandFootObj.motorParameters[1].id, 0x5, HandFootObj.motorParameters[1].homePosition);
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

    //elapseFromLastCall();
    //elapseFromStart();

    std::cout << "Whole Module init done!" << std::endl;

    LfingerObj.startFeedback();
    RfingerObj.startFeedback();
    HandFootObj.startFeedback();

    response = 0;
    std::cout << "Start dB Test? [Y/n]" << std::endl;
    std::cin >> response;

    if(response == 'y' || response == 'Y'){
        SCORE::dbTest(HandFootObj);
        int _isdBTestDone = 0;
        while(!_isdBTestDone){
            SCORE::pressKey(LfingerObj, 11, 0x30);
            usleep(1000000);
            SCORE::releaseKey(LfingerObj, 11, 0x30);
            usleep(1000000);

            SCORE::RPM = 50; //dB test 결과값을 여기에 저장
            _isdBTestDone=1;
        }
        std::cout << "dB Test done!" << std::endl;
        usleep(1000000);
        int tSleep = SCORE::moveToWhiteKey(HandFootObj, 0, defaultLinearVel, 1, 1) * linearTimeConst;
        std::cout << "Sleep for "<< tSleep << "ms" << std::endl;
        usleep(tSleep * 1000);
    }
    else{
        std::cout << "dB Test skipped" << std::endl;
    }
    

    response = 0;
    std::cout << "Start Playing?" << std::endl;
    std::cin >> response;

    if(response == 'y' || response == 'Y'){
        std::cout << "Start Playing!" << std::endl;
        
        //SCORE::calKey(notes, HandFootObj);
        SCORE::currentPosL = 0;
        SCORE::currentPosR = 0;
        setTimerStartPoint();
        size_t quantity = notes.size();
        int octave_tmp = 0;
        while(1){
            for(size_t j = 0; j<quantity; j++){
                if(notes[j].SP_module < elapseFromStart() && notes[j].moveOctave){//move octave
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    std::cout << j << " ";
                    if(notes[j].ID == 1){
                        octave_tmp = notes[j].octave - 6;
                    }
                    else if(notes[j].ID == 2){
                        octave_tmp = 51-notes[j].octave - 7;
                    }
                    SCORE::moveToWhiteKey(HandFootObj, octave_tmp, defaultLinearVel, notes[j].ID, 1);
                    notes[j].moveOctave = 0;
                    std::cout <<"ID : " << notes[j].ID <<  " Bar : " << notes[j].barNum << " Note : " << notes[j].noteNum << " key : " << notes[j].key  << " duration : " << notes[j].duration<< " octave : " << notes[j].octave  << " timeH : " << notes[j].timeHold << " spF : " << notes[j].SP_finger <<  " spM: " << notes[j].SP_module << " mv : "<<notes[j].moveOctave <<  std::endl;
                }

                if(notes[j].SP_finger < elapseFromStart() && notes[j]._isPlayed == 0 && notes[j]._isPressed == 0){ //press key
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    std::cout << "press " << notes[j].barNum << " " << notes[j].noteNum << " key" << std::endl;
                    notes[j]._isPressed = 1;
                    SCORE::pressNote(LfingerObj, RfingerObj, notes[j].ID, notes[j].key);
                    //press
                    //add to pressed list in vector
                    
                }

                if(notes[j].SP_finger + notes[j].timeHold < elapseFromStart() && notes[j]._isPlayed == 0 && notes[j]._isPressed == 1)//check if holdtime of pressed key is over
                {
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    std::cout << "release " << notes[j].barNum << " " << notes[j].noteNum << " key" << std::endl;
                    notes[j]._isPlayed = 1;
                    notes[j]._isPressed = 0;
                    SCORE::releaseNote(LfingerObj, RfingerObj, notes[j].ID, notes[j].key);
                    //release
                    //remove from pressed list in vector
                }
            }
            //std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
        }
        
    }
    else{
        std::cout << "Playing skipped" << std::endl;
    }

    for (auto& t : threads) {
        t.join();
    }

    while(1){

    }

    return 0;
}
