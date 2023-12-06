#include "nebula_decoders/nebula_decoders_velodyne/velodyne_driver.hpp"

#include "nebula_decoders/nebula_decoders_velodyne/decoders/vlp16_decoder.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/decoders/vlp32_decoder.hpp"
#include "nebula_decoders/nebula_decoders_velodyne/decoders/vls128_decoder.hpp"

//#include "nebula_decoders/nebula_decoders_velodyne/decoders/output_builder.hpp"

namespace nebula
{
namespace drivers
{
VelodyneDriver::VelodyneDriver(
  const std::shared_ptr<drivers::VelodyneSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::VelodyneCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  // initialize proper parser from cloud config's model and echo mode
  driver_status_ = nebula::Status::OK;
  std::cout << "sensor_configuration->sensor_model=" << sensor_configuration->sensor_model
            << std::endl;
  switch (sensor_configuration->sensor_model) {
    case SensorModel::UNKNOWN:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
    case SensorModel::VELODYNE_VLS128:
      scan_decoder_.reset(
        new drivers::vls128::Vls128Decoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::VELODYNE_VLP32:
    case SensorModel::VELODYNE_HDL64:
    case SensorModel::VELODYNE_HDL32:
      scan_decoder_.reset(
        new drivers::vlp32::Vlp32Decoder(sensor_configuration, calibration_configuration));
      break;
    case SensorModel::VELODYNE_VLP16:
      scan_decoder_.reset(
        new drivers::vlp16::Vlp16Decoder(sensor_configuration, calibration_configuration));
      break;
    default:
      driver_status_ = nebula::Status::INVALID_SENSOR_MODEL;
      break;
  }

/*
  /////////////////////

  data_ = std::make_shared<drivers::RawData>(this);
  data_->setup();
  data_->setParameters(
    sensor_configuration->cloud_min_angle, sensor_configuration->cloud_max_angle, 0.0, 2.0 * M_PI);
*/
}

Status VelodyneDriver::SetCalibrationConfiguration(
  const CalibrationConfigurationBase & calibration_configuration)
{
  throw std::runtime_error(
    "SetCalibrationConfiguration. Not yet implemented (" +
    calibration_configuration.calibration_file + ")");
}

std::tuple<drivers::NebulaPointCloudPtr, double> VelodyneDriver::ConvertScanToPointcloud(
  const std::shared_ptr<velodyne_msgs::msg::VelodyneScan> & velodyne_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  if (driver_status_ == nebula::Status::OK) {
    scan_decoder_->reset_pointcloud(velodyne_scan->packets.size());
    for (auto & packet : velodyne_scan->packets) {
      scan_decoder_->unpack(packet);
    }
    pointcloud = scan_decoder_->get_pointcloud();
  } else {
    std::cout << "not ok driver_status_ = " << driver_status_ << std::endl;
  }
  return pointcloud;

}

/*
std::tuple<drivers::NebulaPointCloudPtr, double> VelodyneDriver::ConvertScanToPointcloudAW(
  const std::shared_ptr<velodyne_msgs::msg::VelodyneScan> & velodyne_scan)
{
  std::tuple<drivers::NebulaPointCloudPtr, double> pointcloud;
  if (driver_status_ == nebula::Status::OK) {

    bool activate_xyziradt = false;//velodyne_points_ex_pub_->get_subscription_count() > 0;
    bool activate_xyzir = false;//velodyne_points_pub_->get_subscription_count() > 0;
    bool activate_xyzircaedt = true;//velodyne_points_pub_->get_subscription_count() > 0;

    drivers::OutputBuilder output_builder(
        velodyne_scan->packets.size() * data_->scansPerPacket() + _overflow_buffer.points.size(), *velodyne_scan,
//        velodyne_scan->packets.size() * scan_decoder_->pointsPerPacket() + _overflow_buffer.points.size(), *velodyne_scan,
        activate_xyziradt, activate_xyzir, activate_xyzircaedt);

//    output_builder.set_extract_range(data_->getMinRange(), data_->getMaxRange());
    output_builder.set_extract_range(sensor_configuration_->cloud_min_angle, sensor_configuration_->cloud_max_angle);

    if (activate_xyziradt || activate_xyzir || activate_xyzircaedt) {
      // Add the overflow buffer points
      for (size_t i = 0; i < _overflow_buffer.points.size(); ++i) {
        auto &point = _overflow_buffer.points[i];
        output_builder.addPoint(point.x, point.y, point.z, point.return_type,
//            point.ring, point.azimuth, point.distance, point.intensity, point.time_stamp);
            point.channel, point.azimuth, point.distance, point.intensity, point.time_stamp);
      }
      // Reset overflow buffer
      _overflow_buffer.points.clear();
      _overflow_buffer.width = 0;
      _overflow_buffer.height = 1;

      // Unpack up until the last packet, which contains points over-running the scan cut point
      for (size_t i = 0; i < velodyne_scan->packets.size() - 1; ++i) {
        data_->unpack(velodyne_scan->packets[i], output_builder);
      }

      // Split the points of the last packet between pointcloud and overflow buffer
//      drivers::PointcloudXYZIRADT last_packet_points;
//      drivers::PointcloudXYZIRADT last_packet_points_;
      drivers::PointcloudXYZIRCAEDT last_packet_points;
//      drivers::NebulaPointCloud last_packet_points;
      last_packet_points.pc->points.reserve(data_->scansPerPacket());
      data_->unpack(velodyne_scan->packets.back(), last_packet_points);

      // If it's a partial scan, put all points in the main pointcloud
      int phase = (uint16_t)round(sensor_configuration_->scan_phase*100);
      bool keep_all = false;
      uint16_t last_packet_last_phase = (36000 + (uint16_t)last_packet_points.pc->points.back().azimuth - phase) % 36000;
      uint16_t body_packets_last_phase = (36000 + (uint16_t)output_builder.last_azimuth - phase) % 36000;

      if (body_packets_last_phase < last_packet_last_phase) {
        keep_all = true;
      }

      // If it's a split packet, distribute to overflow buffer or main pointcloud based on azimuth
      for (size_t i = 0; i < last_packet_points.pc->points.size(); ++i) {
        uint16_t current_azimuth = (uint16_t)last_packet_points.pc->points[i].azimuth;
        uint16_t phase_diff = (36000 + current_azimuth - phase) % 36000;
        if ((phase_diff > 18000) || keep_all) {
          auto &point = last_packet_points.pc->points[i];
          //PointXYZIRADT
//          output_builder.addPoint(point.x, point.y, point.z, point.return_type,
//              point.ring, point.azimuth, point.distance, point.intensity, point.time_stamp);
          //PointXYZIRCAEDT
          output_builder.addPoint(point.x, point.y, point.z, point.intensity,
              point.return_type, point.channel, point.azimuth, point.elevation, point.distance, point.time_stamp);
        } else {
          _overflow_buffer.points.push_back(last_packet_points.pc->points[i]);
        }
      }

      last_packet_points.pc->points.clear();
      last_packet_points.pc->width = 0;
      last_packet_points.pc->height = 1;
      _overflow_buffer.width = _overflow_buffer.points.size();
      _overflow_buffer.height = 1;
    }


    if (output_builder.xyzir_is_activated()) {
      auto msg = output_builder.move_xyzir_output();
      if (msg->data.size() == 0) msg->header.stamp = velodyne_scan->packets[0].stamp;
      velodyne_points_pub_->publish(std::move(msg));
    }

    if (output_builder.xyziradt_is_activated()) {
      auto msg = output_builder.move_xyziradt_output();
      if (msg->data.size() == 0) msg->header.stamp = velodyne_scan->packets[0].stamp;
      velodyne_points_ex_pub_->publish(std::move(msg));
    }

    if (marker_array_pub_->get_subscription_count() > 0) {
      const auto velodyne_model_marker = createVelodyneModelMakerMsg(scanMsg->header);
      marker_array_pub_->publish(velodyne_model_marker);
    }
    pointcloud = std::make_tuple(scan_pc_, velodyne_scan->packets[0].stamp)
  } else {
    std::cout << "not ok driver_status_ = " << driver_status_ << std::endl;
  }
  return pointcloud;
}
*/
Status VelodyneDriver::GetStatus() { return driver_status_; }

}  // namespace drivers
}  // namespace nebula
