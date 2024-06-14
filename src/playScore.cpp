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
#include "dBPort.hpp"


int _isM12Init = 0;
int _isM3Init = 0;
int _isM12PosSaved = 0;

std::string portLeftFingers = "/dev/ttyACM0";
std::string portRightFingers = "/dev/ttyACM1";
std::string portHandFoot = "/dev/ttyACM2";
std::string portDb = "/dev/ttyACM3";

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

    dBPort dBObj(portDb);

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

    std::vector<double> f4   = {100, 100.5016, 90, 90.5017, 80, 80.5017, 75, 75.5017, 70, 70.5017, 60, 60.5017, 50, 50.5017, 40, 40.5017}; // fff, ff, f, mf, mp, p, pp (f4, f4_1)
    std::vector<int> rpm_lower = {30,  30,  30,  30,  30,  30,  30,  30, 30,  30,  30,  30,  30,  30,  30,  30};
    std::vector<int> rpm_upper = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
    std::vector<int> result_rpm = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    response = 0;
    std::cout << "Start dB Test? [Y/n]" << std::endl;
    std::cin >> response;

    if(response == 'y' || response == 'Y'){
        SCORE::dbTest(HandFootObj);
        int _isdBTestDone = 0;
        double DBERROR = 1.5;
        while(!_isdBTestDone){
            // if(dBObj.sendByte(0x0A)){
            //     std::cout << "sent 0x0A" << std::endl;
            //     usleep(500000);
            //     SCORE::pressKey(LfingerObj, 7, 0x30);
            //     int receivedValue = dBObj.getResponse();
            //     std::cout << "Received response: " << receivedValue << std::endl;
            //     SCORE::RPM = receivedValue;
            //     SCORE::releaseKey(LfingerObj, 7, 0x30);
            //     usleep(500000);
            // }
            // else{
            //     std::cout << "failed to send 0x0A" << std::endl;
            // }

            // usleep(500000); //dB test 결과값을 여기에 저장
            // response = 0;
            // std::cout << "Finish dB Test? [Y/n]" << std::endl;
            // std::cin >> response;
            // if(response == 'y') _isdBTestDone=1;

            
            for(unsigned int i = 0; i < f4.size(); i++)
            {
                double prev_l_rpm = 0;
                double prev_u_rpm = 0;
                double prev_l_dB = 0;
                double prev_u_dB = 0;
                do
                {
                    SCORE::RPM = int(rpm_lower[i]);
                    ////// dbMeter //////
                    if(dBObj.sendByte(0x05)){
                        std::cout << "sent 0x05" << std::endl;
                        usleep(500000);
                    }
                    else std::cout << "failed to send 0x05" << std::endl;
                    
                    if (i % 2 == 0)
                        SCORE::pressKey(LfingerObj, 7, SCORE::RPM); // press f4
                    else
                        SCORE::pressKey(LfingerObj, 8, SCORE::RPM); // press f4_1
                    
                    double l_dB = dBObj.getResponse(); // get dB
                    usleep(500000);
                    SCORE::releaseKey(LfingerObj, 7, 0x30);
                    SCORE::releaseKey(LfingerObj, 8, 0x30);

                    SCORE::RPM = int(rpm_upper[i]);
                    std::cout << i << " " << rpm_upper[i] << std::endl;
                    ////// dbMeter //////
                    
                    if(dBObj.sendByte(0x05)){
                        std::cout << "sent 0x05" << std::endl;
                        usleep(500000);
                    }
                    else std::cout << "failed to send 0x05" << std::endl;
                    
                    if (i % 2 == 0)
                        SCORE::pressKey(LfingerObj, 7, SCORE::RPM); // press f4
                    else
                        SCORE::pressKey(LfingerObj, 8, SCORE::RPM); // press f4_1
                    
                    double u_dB = dBObj.getResponse();; // get dB
                    usleep(500000);
                    SCORE::releaseKey(LfingerObj, 7, 0x30);
                    SCORE::releaseKey(LfingerObj, 8, 0x30);
                    if (l_dB - f4[i] > DBERROR)
                    {
                        rpm_lower[i] = prev_l_rpm;
                        prev_l_dB = l_dB;
                    }
                    if (u_dB - f4[i] < DBERROR)
                    {
                        rpm_upper[i] = prev_u_rpm;
                        prev_u_dB = u_dB;
                    }

                    int mid_rpm = int((rpm_lower[i] + rpm_upper[i])/2.0);

                    prev_l_rpm = rpm_lower[i];
                    prev_u_rpm = rpm_upper[i];
                    prev_l_dB = l_dB;
                    prev_u_dB = u_dB;
                    
                    if ((l_dB + u_dB)/2 >= f4[i]) 
                    {
                        rpm_upper[i] = int((mid_rpm + rpm_upper[i])/2);
                        rpm_lower[i] = mid_rpm;
                    }
                    else
                    {
                        rpm_lower[i] = int((mid_rpm + rpm_lower[i])/2);
                        rpm_upper[i] = mid_rpm;
                    }
                }
                while ((abs(prev_l_dB - f4[i]) > DBERROR) || (abs(prev_u_dB - f4[i]) > DBERROR));

                if (abs(prev_l_dB - f4[i]) > abs(prev_u_dB - f4[i]))
                    result_rpm[i] = prev_u_rpm;
                else
                    result_rpm[i] = prev_l_rpm;

            }
            SCORE::RPM = 50; //dB test 결과값을 여기에 저장
            _isdBTestDone=1;
        }
        std::cout << "dB Test done!" << std::endl;
        for(auto &rpm : result_rpm){
            std::cout << rpm << " " ;

        }
            std::cout << std::endl;
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
        
        SCORE::calKey(notes, HandFootObj);
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
