
if (WIN32)
    if (VCPKG_TARGET_TRIPLET)
        set(OpenCV_DIR "${VCPKG_INSTALLED_DIR}/x64-windows/share/opencv4")
    else ()
        set(OpenCV_DIR "$ENV{OpenCV_DIR}")
    endif ()
endif()

find_package(OpenCV REQUIRED)

add_subdirectory(third_party/aruco)

# data files needs to be placed in a location fixed to the executable
file(COPY "../data" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

add_executable(hello_cv hello_cv.cpp)
target_link_libraries(hello_cv PRIVATE ${OpenCV_LIBRARIES})

add_executable(webcam webcam.cpp)
target_link_libraries(webcam PRIVATE ${OpenCV_LIBRARIES})

add_executable(haarcascade haarcascade.cpp)
target_link_libraries(haarcascade PRIVATE ${OpenCV_LIBRARIES})

add_executable(canny canny.cpp)
target_link_libraries(canny PRIVATE ${OpenCV_LIBRARIES})

add_executable(arcuo_webcam aruco_webcam.cpp)
target_link_libraries(arcuo_webcam PRIVATE ${OpenCV_LIBRARIES} aruco)
