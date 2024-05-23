cmake_minimum_required(VERSION 3.0.1)
project(PIANO_V2)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pkg-config 모듈 찾기
find_package(PkgConfig REQUIRED)

# libserialport 찾기
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# TinyXML2 디렉토리를 추가
add_subdirectory(tinyxml2)

# pthread 라이브러리 찾기
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

# libserialport 및 pthread 라이브러리 링크
target_link_libraries(${PROJECT_NAME} ${LIBSERIALPORT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} tinyxml2)

# include 디렉토리 설정
target_include_directories(${PROJECT_NAME} PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS} include tinyxml2)
