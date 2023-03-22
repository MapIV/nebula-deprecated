# Hw Interface
ament_auto_add_library(velodyne_hw_ros_wrapper SHARED
        velodyne_hw_interface_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_hw_ros_wrapper
        PLUGIN "VelodyneHwInterfaceRosWrapper"
        EXECUTABLE velodyne_hw_ros_wrapper_node
        )


# Monitor
ament_auto_add_library(velodyne_hw_monitor_ros_wrapper SHARED
        velodyne_hw_monitor_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_hw_monitor_ros_wrapper
        PLUGIN "VelodyneHwMonitorRosWrapper"
        EXECUTABLE velodyne_hw_monitor_ros_wrapper_node
        )

# DriverDecoder
ament_auto_add_library(velodyne_driver_ros_wrapper SHARED
        velodyne_decoder_ros_wrapper.cpp
        )
rclcpp_components_register_node(velodyne_driver_ros_wrapper
        PLUGIN "VelodyneDriverRosWrapper"
        EXECUTABLE velodyne_driver_ros_wrapper_node
        )