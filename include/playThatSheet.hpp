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

        uint8_t goalPosition = 0;
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

    void moveToOctave(FingerMotor &motor, uint8_t octave, uint8_t rpm, uint8_t ID) {
        if (octave > motor.g_deviceQuantity || octave < 0) {
            std::cerr << "Invalid Octave" << std::endl;
            return;
        }

        motor.motorParameters[ID];

        //motor.setPosition(octave, 50, goalPosition);
    }

    /// @brief db 테스트 함수
    void dbTest(FingerMotor &motor) {
        std::cout << "dB Test start" << std::endl;
        usleep(1000000);
        motor.setPosition(1, 0x30, )
    }

}

#endif // !PLAYTHATSHEET_H
