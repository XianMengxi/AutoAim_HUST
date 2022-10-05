find_package(OpenCV 4 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(include)
include_directories(/usr/include/eigen3)
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../utils/include)
include_directories(../pose_estimate/include)
include_directories(../lcm)
add_library(buff
        src/buffClassifier.cpp
        src/buffPredictor.cpp
        src/buffCurver.cpp
        src/buffDetector.cpp
        src/buffAngleKalman.cpp
        src/buffCompensation.cpp
        )

        # TX2上要用/usr/local/lib/libceres.a，否则报错
        # 自己电脑上可用${CERES_LIBRARIES}
target_link_libraries(buff ${OpenCV_LIBS} ${CERES_LIBRARIES} lcm_module utils)
