cmake_minimum_required(VERSION 3.0.1)
project(PIANO_V2)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pkg-config 모듈 찾기
find_package(PkgConfig REQUIRED)

# libserialport 찾기
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# pthread 라이브러리 찾기
find_package(Threads REQUIRED)


add_executable(${PROJECT_NAME} main.cpp)

# libserialport 및 pthread 라이브러리 링크
target_link_libraries(PIANO_V2 ${LIBSERIALPORT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

# include 디렉토리 설정
target_include_directories(PIANO_V2 PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS} include)