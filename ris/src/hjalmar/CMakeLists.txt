add_executable(hjalmar
    main.cpp

)

add_dependencies(hjalmar ${catkin_EXPORTED_TARGETS})

target_include_directories(hjalmar PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hjalmar
    ${catkin_LIBRARIES}

    atomic
)

install(
    TARGETS hjalmar
    EXPORT hjalmar
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
