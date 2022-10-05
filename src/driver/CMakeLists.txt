find_package(OpenCV 4 REQUIRED)

include_directories(
        include
        ../utils/include
        ../armor_detector/include
        ${OpenCV_INCLUDE_DIRS}
        
)

add_library(driver
        src/SerialPort.cpp
        src/VideoCapture.cpp
        src/GxCamera.cpp
)

target_link_libraries(driver
        utils
        -lpthread
        -lboost_system
        libgxiapi.so
        ${OpenCV_LIBS}
)