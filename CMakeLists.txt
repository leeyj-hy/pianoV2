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

# main.cpp 및 another.cpp를 실행파일로 추가
add_executable(setZero_exe setZero.cpp)
add_executable(playScore_exe playScore.cpp)
add_executable(DHtest_exe DHtest.cpp)

# include 디렉토리 설정
target_include_directories(setZero_exe PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS} include tinyxml2)
target_include_directories(playScore_exe PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS} include tinyxml2)
target_include_directories(DHtest_exe PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS} include tinyxml2)

# libserialport 및 pthread 라이브러리 링크
target_link_libraries(setZero_exe ${LIBSERIALPORT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} tinyxml2)
target_link_libraries(playScore_exe ${LIBSERIALPORT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} tinyxml2)
target_link_libraries(DHtest_exe ${LIBSERIALPORT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} tinyxml2)

# 빌드 후 .o 파일을 생성하도록 설정
set_target_properties(setZero_exe PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exe
  OUTPUT_NAME "setZero"
)

set_target_properties(playScore_exe PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exe
  OUTPUT_NAME "playScore"
)

set_target_properties(DHtest_exe PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/exe
  OUTPUT_NAME "DHtest"
)