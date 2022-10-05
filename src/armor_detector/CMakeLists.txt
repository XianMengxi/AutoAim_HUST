find_package(OpenCV 4 REQUIRED)
include_directories(
        ./include
        ../utils/include
        ../driver/include
        ../pose_estimate/include
        ../buff/include
        ${OpenCV_INCLUDE_DIRS}
        ../lcm/
        ../AntiRot/include
        /usr/include/eigen3
)

add_library(armor_detector
        src/Detector.cpp
        src/LightBarFinder.cpp
        src/ArmorFinder.cpp
        src/TargetBumper.cpp
        src/NumberClassifier.cpp
        src/DetectAccelerator.cpp
        )

target_link_libraries(armor_detector
        utils
        pose_estimate
        buff
        -lpthread
        ${OpenCV_LIBS} 
)
