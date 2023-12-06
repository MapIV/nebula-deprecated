#include "nebula_ros/velodyne/velodyne_decoder_ros_wrapper.hpp"

#define USE_AWF

namespace nebula
{
namespace ros
{
VelodyneDriverRosWrapper::VelodyneDriverRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("velodyne_driver_ros_wrapper", options)
{
  drivers::VelodyneCalibrationConfiguration calibration_configuration;
  drivers::VelodyneSensorConfiguration sensor_configuration;

  wrapper_status_ = GetParameters(sensor_configuration, calibration_configuration);
  if (Status::OK != wrapper_status_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), this->get_name() << " Error:" << wrapper_status_);
    return;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Starting...");

  calibration_cfg_ptr_ =
    std::make_shared<drivers::VelodyneCalibrationConfiguration>(calibration_configuration);

  sensor_cfg_ptr_ = std::make_shared<drivers::VelodyneSensorConfiguration>(sensor_configuration);

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << ". Driver ");
  wrapper_status_ = InitializeDriver(
    std::const_pointer_cast<drivers::SensorConfigurationBase>(sensor_cfg_ptr_),
    std::static_pointer_cast<drivers::CalibrationConfigurationBase>(calibration_cfg_ptr_));

  RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << "Wrapper=" << wrapper_status_);

//  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
#ifndef USE_AWF
  velodyne_scan_sub_ = create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&VelodyneDriverRosWrapper::ReceiveScanMsgCallback, this, std::placeholders::_1));
#else
  velodyne_scan_sub_ = create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&VelodyneDriverRosWrapper::ReceiveScanMsgCallbackAW, this, std::placeholders::_1));
//  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
#endif
  nebula_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::SensorDataQoS());
  aw_points_base_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", rclcpp::SensorDataQoS());
  aw_points_ex_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", rclcpp::SensorDataQoS());
//  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  /////////////////////

  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

  data_ = std::make_shared<drivers::RawData>(this);
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  data_->setup();
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
  data_->setParameters(
    sensor_configuration.cloud_min_angle * 100, sensor_configuration.cloud_max_angle * 100, 0.0, 2.0 * M_PI);
//    sensor_configuration.cloud_min_angle, sensor_configuration.cloud_max_angle, 0.0, 2.0 * M_PI);
  std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

}

void VelodyneDriverRosWrapper::ReceiveScanMsgCallback(
  const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  // take packets out of scan msg
  std::vector<velodyne_msgs::msg::VelodynePacket> pkt_msgs = scan_msg->packets;

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts =
    driver_ptr_->ConvertScanToPointcloud(scan_msg);
  nebula::drivers::NebulaPointCloudPtr pointcloud = std::get<0>(pointcloud_ts);
  double cloud_stamp = std::get<1>(pointcloud_ts);
  if (pointcloud == nullptr) {
    RCLCPP_WARN_STREAM(get_logger(), "Empty cloud parsed.");
    return;
  };
  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*pointcloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_cloud_xyzi =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIR(pointcloud);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_cloud_xyzi, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    const auto autoware_ex_cloud =
      nebula::drivers::convertPointXYZIRCAEDTToPointXYZIRADT(pointcloud, cloud_stamp);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*autoware_ex_cloud, *ros_pc_msg_ptr);
    ros_pc_msg_ptr->header.stamp =
      rclcpp::Time(SecondsToChronoNanoSeconds(std::get<1>(pointcloud_ts)).count());
    PublishCloud(std::move(ros_pc_msg_ptr), aw_points_ex_pub_);
  }

  auto runtime = std::chrono::high_resolution_clock::now() - t_start;
  RCLCPP_DEBUG(get_logger(), "PROFILING {'d_total': %lu, 'n_out': %lu}", runtime.count(), pointcloud->size());
}

void VelodyneDriverRosWrapper::ReceiveScanMsgCallbackAW(
  const velodyne_msgs::msg::VelodyneScan::SharedPtr scan_msg)
{
  auto t_start = std::chrono::high_resolution_clock::now();


  bool activate_xyziradt = aw_points_ex_pub_->get_subscription_count() > 0;
  bool activate_xyzir = aw_points_base_pub_->get_subscription_count() > 0;
  bool activate_xyzircaedt = nebula_points_pub_->get_subscription_count() > 0;

  drivers::OutputBuilder output_builder(
      scan_msg->packets.size() * data_->scansPerPacket() + _overflow_buffer.points.size(), *scan_msg,
//        velodyne_scan->packets.size() * scan_decoder_->pointsPerPacket() + _overflow_buffer.points.size(), *velodyne_scan,
      activate_xyziradt, activate_xyzir, activate_xyzircaedt);

  output_builder.set_extract_range(data_->getMinRange(), data_->getMaxRange());
//  output_builder.set_extract_range(sensor_configuration_->cloud_min_angle, sensor_configuration_->cloud_max_angle);

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
    for (size_t i = 0; i < scan_msg->packets.size() - 1; ++i) {
      data_->unpack(scan_msg->packets[i], output_builder);
    }

    // Split the points of the last packet between pointcloud and overflow buffer
//      drivers::PointcloudXYZIRADT last_packet_points;
//      drivers::PointcloudXYZIRADT last_packet_points_;
    drivers::PointcloudXYZIRCAEDT last_packet_points;
//      drivers::NebulaPointCloud last_packet_points;
    last_packet_points.pc->points.reserve(data_->scansPerPacket());
    data_->unpack(scan_msg->packets.back(), last_packet_points);

    // If it's a partial scan, put all points in the main pointcloud
    int phase = (uint16_t)round(std::static_pointer_cast<drivers::VelodyneSensorConfiguration>(sensor_cfg_ptr_)->scan_phase*100);
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
    if (msg->data.size() == 0) msg->header.stamp = scan_msg->packets[0].stamp;
    PublishCloud(std::move(msg), aw_points_ex_pub_);
  }

  if (output_builder.xyziradt_is_activated()) {
    auto msg = output_builder.move_xyziradt_output();
    if (msg->data.size() == 0) msg->header.stamp = scan_msg->packets[0].stamp;
    PublishCloud(std::move(msg), aw_points_base_pub_);
  }

  if (output_builder.xyzircaedt_is_activated()) {
    auto msg = output_builder.move_xyzircaedt_output();
    if (msg->data.size() == 0) msg->header.stamp = scan_msg->packets[0].stamp;
    PublishCloud(std::move(msg), nebula_points_pub_);
  }
/*
  if (marker_array_pub_->get_subscription_count() > 0) {
    const auto velodyne_model_marker = createVelodyneModelMakerMsg(scanMsg->header);
    marker_array_pub_->publish(velodyne_model_marker);
  }
*/

  auto runtime = std::chrono::high_resolution_clock::now() - t_start;
  RCLCPP_DEBUG(get_logger(), "PROFILING {'d_total': %lu}", runtime.count());
}

void VelodyneDriverRosWrapper::PublishCloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Timestamp error, verify clock source.");
  }
  pointcloud->header.frame_id = sensor_cfg_ptr_->frame_id;
  publisher->publish(std::move(pointcloud));
}

Status VelodyneDriverRosWrapper::InitializeDriver(
  std::shared_ptr<drivers::SensorConfigurationBase> sensor_configuration,
  std::shared_ptr<drivers::CalibrationConfigurationBase> calibration_configuration)
{
  // driver should be initialized here with proper decoder
  driver_ptr_ = std::make_shared<drivers::VelodyneDriver>(
    std::static_pointer_cast<drivers::VelodyneSensorConfiguration>(sensor_configuration),
    std::static_pointer_cast<drivers::VelodyneCalibrationConfiguration>(calibration_configuration));
  return driver_ptr_->GetStatus();
}

Status VelodyneDriverRosWrapper::GetStatus() { return wrapper_status_; }

Status VelodyneDriverRosWrapper::GetParameters(
  drivers::VelodyneSensorConfiguration & sensor_configuration,
  drivers::VelodyneCalibrationConfiguration & calibration_configuration)
{
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("sensor_model", "");
    sensor_configuration.sensor_model =
      nebula::drivers::SensorModelFromString(this->get_parameter("sensor_model").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("return_mode", "", descriptor);
    sensor_configuration.return_mode =
      nebula::drivers::ReturnModeFromString(this->get_parameter("return_mode").as_string());
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("frame_id", "velodyne", descriptor);
    sensor_configuration.frame_id = this->get_parameter("frame_id").as_string();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "Angle where scans begin (degrees, [0.,360.]";
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0).set__to_value(360).set__step(0.01);
    descriptor.floating_point_range = {range};
    this->declare_parameter<double>("scan_phase", 0., descriptor);
    sensor_configuration.scan_phase = this->get_parameter("scan_phase").as_double();
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    descriptor.read_only = true;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<std::string>("calibration_file", "", descriptor);
    calibration_configuration.calibration_file =
      this->get_parameter("calibration_file").as_string();
  }

  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("min_range", 0.3, descriptor);
    sensor_configuration.min_range = this->get_parameter("min_range").as_double();
  }
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("max_range", 300., descriptor);
    sensor_configuration.max_range = this->get_parameter("max_range").as_double();
  }
  double view_direction;
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("view_direction", 0., descriptor);
    view_direction = this->get_parameter("view_direction").as_double();
  }
  double view_width = 2.0 * M_PI;
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.read_only = false;
    descriptor.dynamic_typing = false;
    descriptor.additional_constraints = "";
    this->declare_parameter<double>("view_width", 2.0 * M_PI, descriptor);
    view_width = this->get_parameter("view_width").as_double();
  }

  if (sensor_configuration.sensor_model != nebula::drivers::SensorModel::VELODYNE_HDL64) {
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      descriptor.read_only = false;
      descriptor.dynamic_typing = false;
      descriptor.additional_constraints = "";
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(0).set__to_value(360).set__step(1);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("cloud_min_angle", 0, descriptor);
      sensor_configuration.cloud_min_angle = this->get_parameter("cloud_min_angle").as_int();
    }
    {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
      descriptor.read_only = false;
      descriptor.dynamic_typing = false;
      descriptor.additional_constraints = "";
      rcl_interfaces::msg::IntegerRange range;
      range.set__from_value(0).set__to_value(360).set__step(1);
      descriptor.integer_range = {range};
      this->declare_parameter<uint16_t>("cloud_max_angle", 360, descriptor);
      sensor_configuration.cloud_max_angle = this->get_parameter("cloud_max_angle").as_int();
    }
  } else {
    double min_angle = fmod(fmod(view_direction + view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    double max_angle = fmod(fmod(view_direction - view_width / 2, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    sensor_configuration.cloud_min_angle = 100 * (2 * M_PI - min_angle) * 180 / M_PI + 0.5;
    sensor_configuration.cloud_max_angle = 100 * (2 * M_PI - max_angle) * 180 / M_PI + 0.5;
    if (sensor_configuration.cloud_min_angle == sensor_configuration.cloud_max_angle) {
      // avoid returning empty cloud if min_angle = max_angle
      sensor_configuration.cloud_min_angle = 0;
      sensor_configuration.cloud_max_angle = 36000;
    }
  }

  if (sensor_configuration.sensor_model == nebula::drivers::SensorModel::UNKNOWN) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (sensor_configuration.return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (sensor_configuration.frame_id.empty() || sensor_configuration.scan_phase > 360) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (calibration_configuration.calibration_file.empty()) {
    return Status::INVALID_CALIBRATION_FILE;
  } else {
    auto cal_status =
      calibration_configuration.LoadFromFile(calibration_configuration.calibration_file);
    if (cal_status != Status::OK) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Given Calibration File: '" << calibration_configuration.calibration_file << "'");
      return cal_status;
    }
  }

  RCLCPP_INFO_STREAM(
    this->get_logger(), "Sensor model: " << sensor_configuration.sensor_model
                                         << ", Return mode: " << sensor_configuration.return_mode
                                         << ", Scan Phase: " << sensor_configuration.scan_phase);
  return Status::OK;
}

RCLCPP_COMPONENTS_REGISTER_NODE(VelodyneDriverRosWrapper)
}  // namespace ros
}  // namespace nebula
