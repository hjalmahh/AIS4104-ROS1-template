find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(pose_predictor
        main.cpp
        kalmanFilter.cpp
        KalmanFilter.h
)

add_dependencies(pose_predictor ${catkin_EXPORTED_TARGETS})

target_include_directories(pose_predictor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS} "/usr/include/eigen3")

target_link_libraries(pose_predictor
        ${catkin_LIBRARIES}

        Eigen3::Eigen
        atomic
)

install(
        TARGETS pose_predictor
        EXPORT pose_predictor
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)