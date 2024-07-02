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

std::string XMLScore = "../../Score/sonata_noA.xml";



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
            usleep(3000000);  // 1sec
            SCORE::releasePedal(HandFootObj, 0x30);
            usleep(2000000);  // 5sec
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

    std::vector<double> f4   =   {85, 85, 60, 60, 55, 55}; // fff, ff, f, mf, mp, p, pp (f4, f4_1)
    std::vector<int> rpm_lower = {8,  8,  8,  8,  8,  8};
    std::vector<int> rpm_upper = {200, 200, 200, 200, 200, 200};

    // std::vector<double> f4   =   {84.2, 84.6, 74.2, 74.6, 68.2, 69.6, 64.2, 64.6}; // fff, ff, f, mf, mp, p, pp (f4, f4_1)
    // std::vector<int> rpm_lower = {30,   30,   30,   30,   30,   30,   30,   30};
    // std::vector<int> rpm_upper = {255,  255,  255,  255,  255,  255,  255,  255};
    // std::vector<double> f4 = {90.0};
    // std::vector<int> rpm_lower = {3};
    // std::vector<int> rpm_upper = {255};
    
    std::vector<int> result_rpm = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    response = 0;
    std::cout << "Start dB Test? [Y/n]" << std::endl;
    std::cin >> response;

    if(response == 'y' || response == 'Y'){
        SCORE::currentPosL = 0;
        SCORE::currentPosR = 0;
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

            // // std::vector<int> rpm_limit = {8};
            // std::vector<int> rpm_limit = {190, 8};
            // int key_num = 7;
            // usleep(500000);
            
            // for (unsigned int i=0; i<rpm_limit.size(); i++)
            // {
            //     std::cout << "rpm : " << rpm_limit[i] << std::endl;
            //     char _r = 0;
            //     std::cout << "next dB Test? [Y/n]" << std::endl;
            //     std::cin >> _r;
            //     if (_r == 'y' || _r == 'Y')
            //     {
            //         double sum_db = 0;

            //         for (int j=0; j<5; j++)
            //         {
            //             SCORE::RPM = rpm_limit[i];
            //             if(dBObj.sendByte(0x08)){
            //                 std::cout << "sent 0x05" << std::endl;
            //                 usleep(500000);
            //             }
            //             else std::cout << "failed to send 0x05" << std::endl;

            //             SCORE::pressKey(LfingerObj, key_num, SCORE::RPM); // press f4
            //             usleep(6000000);

            //             auto _db = dBObj.getResponse();
            //             std::cout << "dB : " << double(_db/100.0) << std::endl; // get dB
            //             sum_db += double(_db/100.0);
            //             SCORE::releaseKey(LfingerObj, key_num, 0x30);
            //             usleep(2000000);
            //             SCORE::releaseKey(LfingerObj, key_num, 0x30);
            //             usleep(500000);

            //         }
            //         std::cout << "average dB : " << sum_db/5 << std::endl;
            //     }
            // }

            // key_num = 8;
            // rpm_limit[0] = 180;
            // for (unsigned int i=0; i<rpm_limit.size(); i++)
            // {
            //     double sum_db = 0;
            //     std::cout << "rpm : " << rpm_limit[i] << std::endl;
            //     char _r = 0;
            //     std::cout << "next dB Test? [Y/n]" << std::endl;
            //     std::cin >> _r;
            //     if (_r == 'y' || _r == 'Y')
            //     {
            //         for (int j=0; j<5; j++)
            //         {
            //             SCORE::RPM = rpm_limit[i];
            //             if(dBObj.sendByte(0x08)){
            //                 std::cout << "sent 0x05" << std::endl;
            //                 usleep(500000);
            //             }
            //             else std::cout << "failed to send 0x05" << std::endl;
                        
            //             SCORE::pressKey(LfingerObj, key_num, SCORE::RPM); // press f4
            //             usleep(6000000);

            //             auto _db = dBObj.getResponse();
            //             std::cout << "dB : " << double(_db/100.0) << std::endl; // get dB
            //             sum_db += double(_db/100.0);
            //             SCORE::releaseKey(LfingerObj, key_num, 0x30);
            //             usleep(2000000);
            //             SCORE::releaseKey(LfingerObj, key_num, 0x30);
            //             usleep(500000);

            //         }
            //         std::cout << "average dB : " << sum_db/5 << std::endl;
            //     }

            // }
            int _p = 0;
            std::cout << "______ dB Test? [Y/n]" << std::endl;
            std::cin >> _p;
            usleep(10000000);  // 5sec
            for(unsigned int i = 0; i < f4.size(); i++)
            {
                double prev_l_rpm = 0;
                double prev_u_rpm = 0;
                double l_dB = 0;
                double u_dB = 0;
                double m_dB = 0;
                int key_num = 0;
                bool start = true;
                bool l_skip = false;
                bool u_skip = false;
                

                std::cout << "f4 : " << f4[i] << std::endl;
                
                if (i%2 == 0)
                    key_num = 7;
                else
                    key_num = 8;
                do
                {
                    int mid_rpm = int((rpm_lower[i] + rpm_upper[i])/2.0);
                    std::vector<int> db_rpm = {rpm_lower[i], rpm_upper[i], mid_rpm};

                    for (unsigned int i=0; i<db_rpm.size(); i++)
                    {
                        if (!start && (i==0 || i==1))
                            continue;
                            
                        SCORE::RPM = db_rpm[i];
                        if(dBObj.sendByte(0x08)){
                            std::cout << "sent 0x05" << std::endl;
                            usleep(500000);
                        }
                        else std::cout << "failed to send 0x05" << std::endl;
                        
                        SCORE::pressKey(LfingerObj, key_num, SCORE::RPM); // press f4
                        usleep(6000000);

                        auto _db = dBObj.getResponse();
                        if (i==0)
                            l_dB = double(_db/100.0); // get dB
                        else if (i==1)
                            u_dB = double(_db/100.0); // get dB
                        else if (i==2)
                            m_dB = double(_db/100.0); // get dB
                        SCORE::releaseKey(LfingerObj, key_num, 0x30);
                        usleep(2000000);
                        SCORE::releaseKey(LfingerObj, key_num, 0x30);
                        usleep(500000);
                    }

                    std::cout << "loewr dB : " << l_dB << " middle dB : " << m_dB << " upper dB : " << u_dB << std::endl;
                    std::cout << "lower rpm : " << db_rpm[0] << " middle rpm : " << db_rpm[2] << " upper rpm : " << db_rpm[1] << std::endl;

                    if (start)
                        start = false;
                    if (abs(l_dB - f4[i]) < DBERROR || abs(u_dB - f4[i]) < DBERROR || abs(m_dB - f4[i]) < DBERROR)
                    {
                        if (abs(l_dB - f4[i]) < DBERROR)
                            result_rpm[i] = db_rpm[0];
                        else if (abs(u_dB - f4[i]) < DBERROR)
                            result_rpm[i] = db_rpm[1];
                        else if (abs(m_dB - f4[i]) < DBERROR)
                            result_rpm[i] = db_rpm[2];
                        break;
                    }
                    else
                    {
                        if (m_dB > f4[i])
                        {
                            rpm_upper[i] = db_rpm[2];

                        }
                        else
                            rpm_lower[i] = db_rpm[2];
                    }
                }
                while (true);
                std::cout << "result rpm : " << result_rpm[i] << std::endl;
                if (i%2 == 0)
                {
                    for (int j=i+1; j<rpm_upper.size(); j++)
                    {
                        if (j%2 == 1)
                            continue;
                        rpm_upper[j] = result_rpm[i];
                    }
                }
                else if (i%2 == 1)
                {
                    for (int j=i+1; j<rpm_upper.size(); j++)
                    {
                        if (j%2 == 0)
                            continue;
                        rpm_upper[j] = result_rpm[i];
                    }
                }
            }
            SCORE::RPM = 255; //dB test 결과값을 여기에 저장
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

        SCORE::currentPosL = 0;
        SCORE::currentPosR = 0;
        
        SCORE::calKey(notes, HandFootObj);
        SCORE::currentPosL = 0;
        SCORE::currentPosR = 0;
        setTimerStartPoint();
        size_t quantity = notes.size();
        int octave_tmp = 0;



        while(1){
            for(size_t j = 0; j<quantity; j++){
                if(notes[j].SP_module+30 < elapseFromStart() && notes[j].moveOctave){//move octave
                    
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    std::cout << j << " ";
                    if(notes[j].ID == 1){   //move left module
                        for(int i = 1; i<15; i++)
                        SCORE::releaseKey(LfingerObj, i, 0xff);
                        if(SCORE::currentPosL < notes[j].octave)    {
                            octave_tmp = notes[j].octave - 6;   //to right
                            if(SCORE::isBlack[(notes[j].key - 1) % 12]) octave_tmp = notes[j].octave - 7;
                        }
                        else if(SCORE::currentPosL > notes[j].octave)   octave_tmp = notes[j].octave - 6;   //to left

                        if(octave_tmp + 6 >= 51 - SCORE::currentPosR) {
                            std::cout << "error : octave_tmp: "<< octave_tmp <<" > currentPosR: "<<SCORE::currentPosR << std::endl;
                            while(1){} //error
                        }
                    }
                    else if(notes[j].ID == 2){  //move right module
                        for(int i = 1; i<15; i++)
                        SCORE::releaseKey(RfingerObj, i, 0xff);
                        if(SCORE::currentPosR < notes[j].octave)    octave_tmp = 51-notes[j].octave - 7;
                        else if(SCORE::currentPosR > notes[j].octave)   octave_tmp = 51-notes[j].octave - 7;
                    
                        if(51 - octave_tmp -7 <= SCORE::currentPosL + 6) {
                            std::cout << "error : octave_tmp:" << octave_tmp << " > currentPosL: " << SCORE::currentPosL << std::endl;
                            while(1){} //error
                        }
                    }
                    
                    SCORE::moveToWhiteKey(HandFootObj, octave_tmp, defaultLinearVel, notes[j].ID, 1);
                    notes[j].moveOctave = 0;
                    std::cout <<"ID : " << notes[j].ID <<  " Bar : " << notes[j].barNum << " Note : " << notes[j].noteNum << " key : " << notes[j].key  << " duration : " << notes[j].duration<< " octave : " << notes[j].octave  << " timeH : " << notes[j].timeHold << " spF : " << notes[j].SP_finger <<  " spM: " << notes[j].SP_module << " mv : "<<notes[j].moveOctave <<  std::endl;
                }

                if(notes[j].SP_finger < elapseFromStart() && notes[j]._isPlayed == 0 && notes[j]._isPressed == 0){ //press key
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    std::cout << "press " << notes[j].barNum << " " << notes[j].noteNum << " key" << std::endl;
                    notes[j]._isPressed = 1;
                    SCORE::pressNote(LfingerObj, RfingerObj, notes[j].ID, notes[j].key, notes[j].note_rpm);
                    //press
                    //add to pressed list in vector
                    if(notes[j].pedal == 1){    //press pedal
                        SCORE::pressPedal(HandFootObj, 0x60);
                        std::cout << "press pedal" << std::endl;
                    }
                }

                if(notes[j].SP_finger + notes[j].timeHold + notes[j].timePress < elapseFromStart() && notes[j]._isPlayed == 0 && notes[j]._isPressed == 1)//check if holdtime of pressed key is over
                {
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    std::cout << "release " << notes[j].barNum << " " << notes[j].noteNum << " key" << std::endl;
                    notes[j]._isPlayed = 1;
                    notes[j]._isPressed = 0;
                    SCORE::releaseNote(LfingerObj, RfingerObj, notes[j].ID, notes[j].key, notes[j].note_rpm*notes[j].articulation_p);
                    //release
                    //remove from pressed list in vector

                }

                if(notes[j].SP_finger + notes[j].timeHold / 2  < elapseFromStart()&& notes[j].pedal == 2){
                    std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << std::endl;
                    SCORE::releasePedal(HandFootObj, 0x60);
                    std::cout << "release pedal" << std::endl;
                    notes[j].pedal = 0;
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
