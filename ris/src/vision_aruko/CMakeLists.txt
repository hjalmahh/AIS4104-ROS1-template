
add_subdirectory(cpp)

add_executable(vision_aruko
    main.cpp

)

add_dependencies(vision_aruko ${catkin_EXPORTED_TARGETS} aruco)

target_include_directories(vision_aruko PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(vision_aruko
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    libopencv_aruco
    aruco
    atomic
)

install(
    TARGETS vision_aruko
    EXPORT vision_aruko
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
