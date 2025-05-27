add_executable(hjalmar
        main.cpp
        ../line_segment/src/laser_ros.cpp
        ../line_segment/src/line_detect.cpp
        sub/line_mapping.cpp
)

add_dependencies(hjalmar ${catkin_EXPORTED_TARGETS})

target_include_directories(hjalmar PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ../line_segment/include
        sub
        ${catkin_INCLUDE_DIRS}
)

target_link_libraries(hjalmar
        ${catkin_LIBRARIES}
        atomic
)

install(
        TARGETS hjalmar
        EXPORT hjalmar
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
