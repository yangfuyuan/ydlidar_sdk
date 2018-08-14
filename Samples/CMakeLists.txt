
cmake_minimum_required(VERSION 2.8)
PROJECT(ydlidar_test)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_BINARY_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
)


LINK_DIRECTORIES(${CMAKE_BINARY_DIR})
ADD_EXECUTABLE(${PROJECT_NAME}
               test.cpp)
ADD_EXECUTABLE(${PROJECT_NAME}1
                          test1.cpp)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ydlidar_driver)
TARGET_LINK_LIBRARIES(${PROJECT_NAME}1 ydlidar_driver)

