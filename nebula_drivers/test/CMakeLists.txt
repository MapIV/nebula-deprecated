if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    find_package(ament_cmake_gtest)

    add_definitions(-D_SRC_RESOURCES_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/test/data/")
    add_definitions(-D_SRC_CALIBRATION_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}/calibration/")

    set(NEBULA_TEST_DEPENDENCIES
            rclcpp
            rosbag2_cpp
            pandar_msgs
            )

    add_subdirectory(hesai)
    add_subdirectory(velodyne)

endif()