cmake_minimum_required(VERSION 3.5)
project(AutoAim)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_PREFIX_PATH "/usr/local/opencv/opencv-4.5.1")
SET(CMAKE_BUILD_TYPE "Debug")

find_package(OpenCV 4 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

add_subdirectory(src/armor_detector)
add_subdirectory(src/pose_estimate)
add_subdirectory(src/driver)
add_subdirectory(src/utils)
add_subdirectory(src/buff)
add_subdirectory(src/lcm)


include_directories(
        src/
        src/utils/include
        src/armor_detector/include
        src/buff/include
        src/driver/include
        src/lcm/
        src/pose_estimate/include
        src/AntiRot/include
        /usr/include/eigen3
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(AutoAim
        src/AutoAim.cpp

)

target_link_libraries(AutoAim
        utils
        driver
        armor_detector
        buff
        ${OpenCV_LIBS}
        -lboost_system
        -lboost_filesystem

)

