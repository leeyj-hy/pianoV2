cmake_minimum_required(VERSION 3.0.1)
project(PIANO_V2)

include(ExternalProject)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# pkg-config 모듈 찾기
find_package(PkgConfig REQUIRED)

# libserialport 찾기
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

# TinyXML2 라이브러리 다운로드 및 빌드
ExternalProject_Add(tinyxml2_proj
  GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
  GIT_TAG        9.0.0
  PREFIX         ${CMAKE_BINARY_DIR}/tinyxml2
  CMAKE_ARGS     -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}/tinyxml2/install
)

add_library(tinyxml2 STATIC IMPORTED)
set_target_properties(tinyxml2 PROPERTIES
  IMPORTED_LOCATION ${CMAKE_BINARY_DIR}/tinyxml2/install/lib/libtinyxml2.a
  INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/tinyxml2/install/include
)

add_dependencies(tinyxml2 tinyxml2_proj)

# pthread 라이브러리 찾기
find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

# libserialport 및 pthread 라이브러리 링크
target_link_libraries(${PROJECT_NAME} ${LIBSERIALPORT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} tinyxml2)

# include 디렉토리 설정
target_include_directories(${PROJECT_NAME} PRIVATE ${LIBSERIALPORT_INCLUDE_DIRS} ${CMAKE_BINARY_DIR}/tinyxml2/install/include include)
