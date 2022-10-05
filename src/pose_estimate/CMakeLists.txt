#find_package()
include_directories(
        include
        ../utils/include
        /usr/include/eigen3
        ${Sophus_INCLUDE_DIRS}
        ../lcm/
        ../armor_detector/include
        ../driver/include
        )

add_library(pose_estimate
        src/PoseSolver.cpp
        src/Predictor.cpp
        src/NormalKalman.cpp
        src/SecondFilter.cpp
        src/NormalEKF.cpp
        src/PitchYawFilter.cpp
        )
 
target_link_libraries(pose_estimate
        utils
        /usr/local/lib/libSophus.so
        lcm_module
        ) 
