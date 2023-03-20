## HESAI
# HwInterface Lib
ament_auto_add_library(hesai_hw_interface SHARED
        hesai_hw_interface.cpp
        )
# RosHwInterface
ament_auto_add_library(hesai_hw_ros_wrapper SHARED
        hesai_hw_interface_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_hw_ros_wrapper
        PLUGIN "HesaiHwInterfaceRosWrapper"
        EXECUTABLE hesai_hw_interface_ros_wrapper_node
        )

# RosHwMonitor
ament_auto_add_library(hesai_hw_monitor_ros_wrapper SHARED
        hesai_hw_monitor_ros_wrapper.cpp
        )
target_link_libraries(hesai_hw_monitor_ros_wrapper ${CURL_LIBRARIES})

rclcpp_components_register_node(hesai_hw_monitor_ros_wrapper
        PLUGIN "HesaiHwMonitorRosWrapper"
        EXECUTABLE hesai_hw_monitor_ros_wrapper_node
        )

# Driver Lib
ament_auto_add_library(hesai_driver SHARED
        hesai_driver.cpp
        decoders/pandar_40_decoder.cpp
        decoders/pandar_64_decoder.cpp
        decoders/pandar_qt_64_decoder.cpp
        decoders/pandar_qt_128_decoder.cpp
        decoders/pandar_xt_decoder.cpp
        decoders/pandar_xtm_decoder.cpp
        decoders/pandar_at_decoder.cpp
        decoders/pandar_128_e4x_decoder.cpp
        )

# RosDriver
ament_auto_add_library(hesai_driver_ros_wrapper SHARED
        hesai_driver_ros_wrapper.cpp
        )

rclcpp_components_register_node(hesai_driver_ros_wrapper
        PLUGIN "HesaiDriverRosWrapper"
        EXECUTABLE hesai_driver_ros_wrapper_node
        )
ament_export_libraries(hesai_driver)
