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

#define linearTimeConst 850    //how many ms to move 1 white key
#define defaultLinearVel 0xff   //default velocity

#define posPress 10
#define posRelease -30

namespace SCORE {

    using namespace tinyxml2;
    using namespace std;

    struct NOTE {
        int barNum;
        int noteNum;

        int octave;
        char key;
        float duration;
        int articulation;

        long long SP_finger;    //start point of finger
        long long SP_module;    //start point of module

        long long RP_finger;    //release point of inger
        long long RP_module;    //release point of module
    };

    int currentPosL = 0;
    int currentPosR = 0;

    int loadScoreFromXML(const std::string& filename, std::vector<NOTE>& notes) {
        int retVal = -1;
        XMLDocument doc;
        cout << "Loading Score from " << filename << endl;
        if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
            cout << "Failed to load file" << endl;
            return retVal;
        }

        XMLElement* scorePartwise = doc.FirstChildElement("score-partwise");
        if (!scorePartwise) {
            cout << "<score-partwise> not found" << endl;
            return retVal;
        }
        std::cout << "score found" << std::endl;
        // Read <head> element
        XMLElement* head = scorePartwise->FirstChildElement("head");
        if (head) {
            XMLElement* tempo = head->FirstChildElement("tempo");
            XMLElement* time = head->FirstChildElement("time");
            if (time) {
                XMLElement* beats = time->FirstChildElement("beats");
                XMLElement* beatType = time->FirstChildElement("beat-type");
            }
            XMLElement* defaultRpm = head->FirstChildElement("defaultRpm");
        }
        std::cout << "reading head done" << std::endl;
        // Read <part> element
        XMLElement* part = scorePartwise->FirstChildElement("part");
        if (!part) {
            cout << "<part> not found" << endl;
            return retVal;
        }
        std::cout << "reading part done" << std::endl;
        // Loop through each <BAR> element
        for (XMLElement* bar = part->FirstChildElement("BAR"); bar != nullptr; bar = bar->NextSiblingElement("BAR")) {
            int barNum = bar->IntAttribute("barNum");

            // Loop through each <NOTE> element within the <BAR>
            for (XMLElement* noteElement = bar->FirstChildElement("NOTE"); noteElement != nullptr; noteElement = noteElement->NextSiblingElement("NOTE")) {
                int noteNum = noteElement->IntAttribute("noteNum");
                std::cout << noteNum << std::endl;
                XMLElement* note = noteElement->FirstChildElement("note");
                if (note) {
                    NOTE noteData;
                    noteData.barNum = barNum;
                    noteData.noteNum = noteNum;

                    XMLElement* pitch = note->FirstChildElement("pitch");
                    if (pitch) {
                        XMLElement* octave = pitch->FirstChildElement("octave");
                        XMLElement* key = pitch->FirstChildElement("key");
                        if (octave) noteData.octave = octave->IntText();
                        if (key) noteData.key = key->GetText()[0];
                    }
                    XMLElement* duration = note->FirstChildElement("duration");
                    if (duration) noteData.duration = duration->FloatText();
                    XMLElement* articulation = note->FirstChildElement("articulation");
                    if (articulation) noteData.articulation = articulation->IntText();

                    notes.push_back(noteData);
                }
            }
        }
        
        retVal = 0; // Successfully loaded
        std::cout << "reading bar done" << std::endl;
        return retVal;
    }

    void calKey() {

    }

    /// @brief 키 번호와 rpm을 입력받아 해당 모터를 작동시키는 키 누르기 함수
    /// @param motor : 디바이스 객체
    /// @param key : 키 번호
    /// @param rpm  : 회전 속도
    void pressKey(FingerMotor &motor, uint8_t key, uint8_t rpm) {
        if (key > motor.g_deviceQuantity || key < 0) {
            std::cerr << "Invalid Key" << std::endl;
            return;
        }

        uint8_t goalPosition = posPress;
        motor.setPosition(key, rpm, goalPosition);
    }

    /// @brief 키 번호와 rpm을 입력받아 해당 모터를 작동시키는 키 떼기 함수
    /// @param motor : 디바이스 객체
    /// @param key : 키 번호
    /// @param rpm : 회전 속도
    void releaseKey(FingerMotor &motor, uint8_t key, uint8_t rpm) {
        if (key > motor.g_deviceQuantity || key < 0) {
            std::cerr << "Invalid Key" << std::endl;
            return;
        }
        MotorParameter param = motor.ReadMotorParameterByID(motor.paramFilePath, key);
        uint8_t goalPosition = param.homePosition;

        motor.setPosition(key, rpm, goalPosition);
    }

    /// @brief 옥타브와 rpm을 입력받아 해당 모터를 옥타브로 이동시키는 함수
    /// @param motor 
    /// @param octave 
    /// @param rpm 
    /// @param ID 
    /// @return 움직이는 간격 계산결과값 반환(칸수)
    int moveToOctave(FingerMotor &motor, uint8_t octave, uint8_t rpm, uint8_t ID) {
        int goalPosition = 0;
        int retVal = 0;
        if(ID == 1){
            std::cout << "Left Hand ";
        }
        else if(ID == 2){
            std::cout << "Right Hand ";
        }
        else{
            std::cerr << "Invalid ID" << std::endl;
            return retVal;
        }
        
        if(octave == 0 && ID == 1){
            std::cout << "Move to 0 Octave" << std::endl;
            motor.setPosition(ID, rpm, goalPosition);
            retVal = currentPosL;  //how many steps moved
            currentPosL = 0;
            return retVal;
        }
        else if(octave >0 && octave < 7){
            std::cout << "Move to " << octave << " Octave" << std::endl;
            if(ID == 1){    //case left hand
                goalPosition = 2 + (octave-1)*7;
                if(goalPosition + 7 > currentPosR){
                    std::cerr << "Invalid Position Left" << std::endl;
                    return retVal;//case push
                }
                motor.setPosition(ID, rpm, goalPosition);
                retVal = abs(currentPosL-goalPosition);  //how many steps moved
                currentPosL = goalPosition;
                return retVal;
            }
            else if (ID == 2){  //case right hand
                goalPosition = (7 - octave)*7;
                if(goalPosition - 14 < currentPosL){
                    std::cerr << "Invalid Position Right" << std::endl;
                    return retVal;//case push
                }
                motor.setPosition(ID, rpm, goalPosition);
                retVal = abs(currentPosR-51+goalPosition);  //how many steps moved
                currentPosR = 51 - goalPosition;
                return retVal;
            }
            else{
                std::cerr << "Invalid ID" << std::endl;
                return retVal;
            }
        }
        else if(octave == 7 && ID == 2){
            std::cout << "Move to 7 Octave" << std::endl;
            motor.setPosition(ID, rpm, goalPosition);
            retVal = 51 - currentPosR;  //how many steps moved
            currentPosL = 0;
            return retVal;
        }
        else{
            std::cerr << "Invalid Octave" << std::endl;
            return retVal;
        }
    }

    /// @brief db 테스트 함수
    /// @param motor
    void dbTest(FingerMotor &motor) {
        std::cout << "dB Test start" << std::endl;
        usleep(1000000);
        std::cout << "move to 4th octave" << std::endl;
        int time4sleep = moveToOctave(motor, 4, defaultLinearVel, 1);   //move left hand to 4th octave
        usleep(time4sleep*linearTimeConst);
        std::cout << "ready for octave test" << std::endl;
    }

    void handPressKey(FingerMotor &motor, uint8_t key, uint8_t rpm) {
        pressKey(motor, key, rpm);
    }

}

#endif // !PLAYTHATSHEET_H
