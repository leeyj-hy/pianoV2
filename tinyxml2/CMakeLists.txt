cmake_minimum_required(VERSION 3.0)
project(tinyxml2)

add_library(tinyxml2 STATIC tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
