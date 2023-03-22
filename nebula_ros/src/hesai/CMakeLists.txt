# Hw Interface
ament_auto_add_library(hesai_hw_ros_wrapper SHARED
        hesai_hw_interface_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_hw_ros_wrapper
        PLUGIN "HesaiHwInterfaceRosWrapper"
        EXECUTABLE hesai_hw_interface_ros_wrapper_node
        )

# Monitor
ament_auto_add_library(hesai_hw_monitor_ros_wrapper SHARED
        hesai_hw_monitor_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_hw_monitor_ros_wrapper
        PLUGIN "HesaiHwMonitorRosWrapper"
        EXECUTABLE hesai_hw_monitor_ros_wrapper_node
        )

# DriverDecoder
ament_auto_add_library(hesai_driver_ros_wrapper SHARED
        hesai_decoder_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_driver_ros_wrapper
        PLUGIN "HesaiDriverRosWrapper"
        EXECUTABLE hesai_driver_ros_wrapper_node
        )