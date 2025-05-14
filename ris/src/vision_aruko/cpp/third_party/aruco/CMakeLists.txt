
add_library(aruco
        src/aruco.cpp
        src/aruco_calib.cpp
        src/charuco.cpp
)
target_include_directories(aruco PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(aruco PUBLIC ${OpenCV_LIBRARIES})
