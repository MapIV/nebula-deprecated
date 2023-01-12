#ifndef NEBULA_VelodyneRosOfflineExtractSample_H
#define NEBULA_VelodyneRosOfflineExtractSample_H

#include "common/nebula_common.hpp"
#include "common/nebula_driver_ros_wrapper_base.hpp"
#include "common/nebula_status.hpp"
#include "velodyne/velodyne_common.hpp"
#include "velodyne/velodyne_driver.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

namespace nebula
{
namespace ros
{
class VelodyneRosOfflineExtractSample final : public rclcpp::Node, NebulaDriverRosWrapperBase
{
  std::shared_ptr<drivers::VelodyneDriver> driver_ptr_;
  Status wrapper_status_;
//  rclcpp::Subscription<velodyne_msgs::msg::VelodyneScan>::SharedPtr velodyne_scan_sub_;
//  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_pub_;

  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_cfg_ptr_;
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_cfg_ptr_;

  Status InitializeDriver(
    std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
    std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration) override;

  Status GetParameters(
    drivers::VelodyneSensorConfiguration & sensor_configuration,
    drivers::VelodyneCalibrationConfiguration & calibration_configuration);

  static inline std::chrono::nanoseconds SecondsToChronoNanoSeconds(const double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

public:
  explicit VelodyneRosOfflineExtractSample(
    const rclcpp::NodeOptions & options, const std::string & node_name);

  void ReceiveScanMsgCallback(const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg);
  Status GetStatus();
  Status ReadBag();
private:
  std::string bag_path;
  std::string storage_id;
  std::string out_path;
  std::string format;
  std::string target_topic;
};

}  // namespace ros
}  // namespace nebula

#endif  // NEBULA_VelodyneRosOfflineExtractSample_H
