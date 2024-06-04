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

#define linearTimeConst 85    //how many ms to move 1 white key
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
        int key;
        float duration;
        int articulation;
        int ID;

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
                    XMLElement* ID = note->FirstChildElement("ID");
                    if(ID) noteData.ID = ID->IntText();
                    XMLElement* pitch = note->FirstChildElement("pitch");
                    if (pitch) {
                        XMLElement* octave = pitch->FirstChildElement("octave");
                        XMLElement* key = pitch->FirstChildElement("key");
                        if (octave) noteData.octave = octave->IntText();
                        if (key) noteData.key = key->IntText();
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

    void calKey(std::vector<SCORE::NOTE> notes) {
        for(auto& note :notes){
            
        }
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
        uint8_t goalPosition = 0;
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
            motor.setPosition(ID, rpm, 0);
            retVal = currentPosL;  //how many steps moved
            currentPosL = 0;
            return retVal;
        }
        else if(octave >0 && octave < 7){
            std::cout << "Move to " << (int)octave << " Octave" << std::endl;
            if(ID == 1){    //case left hand
                goalPosition = 2 + (octave-1)*7;
                if(goalPosition + 7 > 51-(currentPosR+7)){
                    std::cerr << "Invalid Position Left" << std::endl;
                    return retVal;//case push
                }
                motor.setPosition(ID, rpm, -goalPosition);
                retVal = abs(currentPosL-goalPosition);  //how many steps moved
                currentPosL = goalPosition;
                return retVal;
            }
            else if (ID == 2){  //case right hand
                goalPosition = (7 - octave)*7;
                if(51-(goalPosition+7) < currentPosL+7){
                    std::cerr << "Invalid Position Right" << std::endl;
                    return retVal;//case push
                }
                motor.setPosition(ID, rpm, -goalPosition);
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
            motor.setPosition(ID, rpm, 0);
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
        std::cout << "Sleep for "<< time4sleep*linearTimeConst << "ms" << std::endl;
        usleep(time4sleep*linearTimeConst*1000);
        std::cout << "ready for octave test" << std::endl;
    }


    /// @brief 기준음이 주어졌을 때 해당 음을 누르는 함수
    /// @param motor : 디바이스 객체
    /// @param currentPos : 현재 손 위치
    /// @param noteH : 누를 음의 높이
    /// @param rpm : 회전 속도
    void pressNote(FingerMotor &motor, int currentPos, int noteH, uint8_t rpm) {
        
    }


    /// @brief 88개의 키 중에서 주어진 키가 몇 번째 백건인지 계산하는 함수
    /// @param fullKey 
    /// @param direction 
    /// @return 
    int fullToWhiteKey(int fullKey, int direction) {
    // 각 키의 백건 여부 판별
    static const bool isWhiteKey[] = {
        true, false, true, false, true,  // C, C#, D, D#, E
        true, false, true, false, true, false, true,  // F, F#, G, G#, A, A#, B
    };

    // 백건 카운트
    int whiteCount = 0;

    // 전체 키를 순회하면서 백건 카운트
    for (int i = 0; i < fullKey; i++) {
        if (isWhiteKey[i % 12]) whiteCount++;
    }

    // 현재 키가 백건이면 바로 반환
    if (isWhiteKey[fullKey % 12]) return whiteCount;

    // 흑건인 경우 direction에 따라 처리
    if (direction == 1) {  // 낮은음의 백건
        // 흑건 전에 나오는 마지막 백건을 찾기
        for (int i = fullKey; i >= 0; i--) {
            if (isWhiteKey[i % 12]) {
                return fullToWhiteKey(i);
            }
        }
    } else if (direction == 2) {  // 높은음의 백건
        // 흑건 후에 나오는 첫 번째 백건을 찾기
        for (int i = fullKey; i < 88; i++) {
            if (isWhiteKey[i % 12]) {
                return fullToWhiteKey(i);
            }
        }
    }

    // 적절한 백건을 찾지 못한 경우 (오류 방지)
    return -1;
    }



}

#endif // !PLAYTHATSHEET_H
