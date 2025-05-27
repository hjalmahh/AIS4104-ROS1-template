add_executable(line_segment
        main.cpp
        src/laser_ros.cpp
        src/line_detect.cpp


)

add_dependencies(line_segment ${catkin_EXPORTED_TARGETS})

target_include_directories(line_segment PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(line_segment
        ${catkin_LIBRARIES}

        atomic
)

install(
        TARGETS line_segment
        EXPORT line_segment
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)