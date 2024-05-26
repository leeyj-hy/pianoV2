#ifndef WHATTIME_H
#define WHATTIME_H

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

atomic<long long> firstupdate = 0;
atomic<long long> timeElapseFromStart = 0;

atomic<long long> lastUpdate = 0;
atomic<long long> timeElapse = 0;

// 전역 변수로 현재 시간을 저장할 변수 선언
std::atomic<long long> current_time_millis(0);

// 현재 시간을 밀리초 단위로 가져와 변수에 저장하는 함수
void updateCurrentTime() {
    while (true) {
        // 현재 시간을 얻기 위한 시스템 클락 사용
        auto now = std::chrono::system_clock::now();

        // 시스템 클락을 밀리초 단위로 변환
        auto duration = now.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

        // 전역 변수에 현재 시간 (밀리초 단위) 저장
        current_time_millis.store(millis);

        // 1초마다 현재 시간을 갱신
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void elapseFromLastCall(){
    timeElapse = current_time_millis - lastUpdate;
    std::cout << "======== Elapse From Last Call : " << timeElapse << " mS ========" << endl;
    lastUpdate.store(current_time_millis);
}

void elapseFromStart(){
    if(firstupdate){
       timeElapseFromStart = current_time_millis - firstupdate;
        std::cout << "======== Elapse From Start : " << timeElapseFromStart << " mS ========" << endl; 
    }
    else{
        std::cout << "======== Timer StartPoint not set ========" << endl;
    }
}

void setTimerStartPoint(){
    if(!firstupdate){
            firstupdate.store(current_time_millis);
            std::cout << "======== Timer Start Point : " << firstupdate << " mS ========" << endl;
        }
    else{
        std::cout << "======== Timer Has Already Started ========" << endl;
    }
}




#endif // !WHATTIME_H
