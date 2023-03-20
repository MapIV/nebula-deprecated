## Velodyne
# HwInterfaces Lib
ament_auto_add_library(velodyne_hw_interface SHARED
        velodyne_hw_interface.cpp
        )
# RosHwInterface
ament_auto_add_library(velodyne_hw_ros_wrapper SHARED
        velodyne_hw_interface_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_hw_ros_wrapper
        PLUGIN "VelodyneHwInterfaceRosWrapper"
        EXECUTABLE velodyne_hw_ros_wrapper_node
        )

target_link_libraries(velodyne_hw_ros_wrapper ${CURL_LIBRARIES})

# RosHwMonitor
ament_auto_add_library(velodyne_hw_monitor_ros_wrapper SHARED
        velodyne_hw_monitor_ros_wrapper.cpp
        )
target_link_libraries(velodyne_hw_monitor_ros_wrapper ${CURL_LIBRARIES})
rclcpp_components_register_node(velodyne_hw_monitor_ros_wrapper
        PLUGIN "VelodyneHwMonitorRosWrapper"
        EXECUTABLE velodyne_hw_monitor_ros_wrapper_node
        )

# Driver Lib
ament_auto_add_library(velodyne_driver SHARED
        velodyne_driver.cpp
        decoders/velodyne_calibration_decoder.cpp
        decoders/vls128_decoder.cpp
        decoders/vlp16_decoder.cpp
        decoders/vlp32_decoder.cpp
        )

target_link_libraries(velodyne_driver ${YAML_CPP_LIBRARIES})
# RosDriver
ament_auto_add_library(velodyne_driver_ros_wrapper SHARED
        velodyne_driver_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_driver_ros_wrapper
        PLUGIN "VelodyneDriverRosWrapper"
        EXECUTABLE velodyne_driver_ros_wrapper_node
        )
ament_export_libraries(velodyne_driver)
