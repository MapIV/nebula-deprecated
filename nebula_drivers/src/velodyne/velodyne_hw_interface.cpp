#include "velodyne/velodyne_hw_interface.hpp"

#include <velodyne_msgs/msg/velodyne_packet.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <memory>

#include <boost/format.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <strstream>

namespace nebula
{
namespace drivers
{
VelodyneHwInterface::VelodyneHwInterface()
: cloud_io_context_{new IoContext(1)},
  cloud_udp_driver_{new ::drivers::udp_driver::UdpDriver(*cloud_io_context_)},
  scan_cloud_ptr_{std::make_unique<velodyne_msgs::msg::VelodyneScan>()},
  boost_ctx_{new boost::asio::io_context()},
  http_client_driver_{new ::drivers::tcp_driver::HttpClientDriver(boost_ctx_)}
{
}

Status VelodyneHwInterface::InitializeSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  sensor_configuration_ = std::static_pointer_cast<VelodyneSensorConfiguration>(sensor_configuration);
  phase_ = (uint16_t)round(sensor_configuration_->scan_phase*100);


//  InitHttpClient();
  // Need to check status
//  GetDiag();
//  GetStatus();
//  GetSnapshot();
  // Need to compare with current settings!!
//  SetRpm(sensor_configuration_->rotation_speed);
//  SetFovStart(sensor_configuration_->cloud_min_angle);
//  SetFovEnd(sensor_configuration_->cloud_max_angle);
//  SetReturnType(sensor_configuration_->return_mode);

//  InitHttpClientAsync();
  GetDiagAsync();
  GetStatusAsync();
  /*
  GetSnapshotAsync();
  // Need to compare with current settings!!
  SetRpmAsync(sensor_configuration_->rotation_speed);
  SetFovStartAsync(sensor_configuration_->cloud_min_angle);
  SetFovEndAsync(sensor_configuration_->cloud_max_angle);
  SetReturnTypeAsync(sensor_configuration_->return_mode);
  boost_ctx_->run();
  */

 /*
  Status st1 = Status::HTTP_CONNECTION_ERROR;
  std::cout << "Status::HTTP_CONNECTION_ERROR : " << st1 << std::endl;
  VelodyneStatus st2 = VelodyneStatus::INVALID_RPM_ERROR;
  std::cout << "VelodyneStatus::INVALID_RPM_ERROR : " << st2 << std::endl;
  Status st3 = st2;
  std::cout << "Status::INVALID_RPM_ERROR : " << st3 << std::endl;
  VelodyneStatus st4 = Status::HTTP_CONNECTION_ERROR;
  std::cout << "VelodyneStatus::HTTP_CONNECTION_ERROR : " << st4 << std::endl;
  std::cout << "st4.type() : " << st4.type() << std::endl;
  Status st5 = st4;
  std::cout << "VelodyneStatus to Status::HTTP_CONNECTION_ERROR : " << st5 << std::endl;
  Status st6 = Status(9);
  std::cout << "Status(9) : " << st6 << std::endl;
  Status st7 = static_cast<Status>(st4);
  std::cout << "VelodyneStatus to static_cast Status::HTTP_CONNECTION_ERROR : " << st5 << std::endl;
  */
  Status status = Status::OK;
  return status;
}

Status VelodyneHwInterface::SetSensorConfiguration(
  std::shared_ptr<SensorConfigurationBase> sensor_configuration)
{
  InitializeSensorConfiguration(sensor_configuration);

//  Status status = Status::OK;
  VelodyneStatus status = CheckAndSetConfigBySnapshotAsync();
  Status rt = status;
  return rt;
}

Status VelodyneHwInterface::CloudInterfaceStart()
{
  try {
//    std::cout << "Starting UDP server on: " << sensor_configuration_->host_ip << std::endl;
    cloud_udp_driver_->init_receiver(
      sensor_configuration_->host_ip, sensor_configuration_->data_port);
    cloud_udp_driver_->receiver()->open();
    cloud_udp_driver_->receiver()->bind();
    cloud_udp_driver_->receiver()->asyncReceive(
      std::bind(&VelodyneHwInterface::ReceiveCloudPacketCallback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << sensor_configuration_->data_port << std::endl;
    return status;
  }
  return Status::OK;
}

Status VelodyneHwInterface::RegisterScanCallback(
  std::function<void(std::unique_ptr<velodyne_msgs::msg::VelodyneScan>)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

void VelodyneHwInterface::ReceiveCloudPacketCallback(const std::vector<uint8_t> & buffer)
{
  // Process current packet
  uint32_t buffer_size = buffer.size();
  std::array<uint8_t, 1206> packet_data{};
  std::copy_n(std::make_move_iterator(buffer.begin()), buffer_size, packet_data.begin());
  velodyne_msgs::msg::VelodynePacket velodyne_packet;
  auto now = std::chrono::system_clock::now();
  auto now_secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
  auto now_nanosecs = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  velodyne_packet.data = packet_data;
  velodyne_packet.stamp.sec = static_cast<int>(now_secs);
  velodyne_packet.stamp.nanosec = static_cast<int>((now_nanosecs/1000000000. - static_cast<double>(now_secs))*1000000000);
  scan_cloud_ptr_->packets.emplace_back(velodyne_packet);
  processed_packets_++;

  // Check if scan is complete
  packet_first_azm_  = scan_cloud_ptr_->packets.back().data[2]; // lower word of azimuth block 0
  packet_first_azm_ |= scan_cloud_ptr_->packets.back().data[3] << 8; // higher word of azimuth block 0

  packet_last_azm_ = scan_cloud_ptr_->packets.back().data[1102];
  packet_last_azm_ |= scan_cloud_ptr_->packets.back().data[1103] << 8;

  packet_first_azm_phased_ = (36000 + packet_first_azm_ - phase_) % 36000;
  packet_last_azm_phased_ = (36000 + packet_last_azm_ - phase_) % 36000;
//  std::cout << "first:" << packet_first_azm_phased_ << std::endl;
//  std::cout << "last" << packet_last_azm_phased_ << std::endl;

  if (processed_packets_ > 1) {
    if (
      packet_last_azm_phased_ < packet_first_azm_phased_ ||
      packet_first_azm_phased_ < prev_packet_first_azm_phased_) {
        // Callback
        scan_reception_callback_(std::move(scan_cloud_ptr_));
        scan_cloud_ptr_ = std::make_unique<velodyne_msgs::msg::VelodyneScan>();
        processed_packets_ = 0;
    }
  }
  prev_packet_first_azm_phased_ = packet_first_azm_phased_;
}
Status VelodyneHwInterface::CloudInterfaceStop() { return Status::ERROR_1; }

Status VelodyneHwInterface::GetSensorConfiguration(SensorConfigurationBase & sensor_configuration)
{
  std::stringstream ss;
  ss << sensor_configuration;
  PrintDebug(ss.str());
  return Status::ERROR_1;
}

Status VelodyneHwInterface::GetCalibrationConfiguration(
  CalibrationConfigurationBase & calibration_configuration)
{
  PrintDebug(calibration_configuration.calibration_file);
  return Status::ERROR_1;
}



VelodyneStatus VelodyneHwInterface::InitHttpClient()
{
  try {
    http_client_driver_->init_client(sensor_configuration_->sensor_ip, 80);
    if (!http_client_driver_->client()->isOpen()) {
      http_client_driver_->client()->open();
    }
  } catch (const std::exception & ex) {
//    Status status = Status::HTTP_CONNECTION_ERROR;
    VelodyneStatus status = Status::HTTP_CONNECTION_ERROR;
//    std::cerr << status << sensor_configuration_->sensor_ip << ","
//              << 80 << std::endl;
    return status;
  }
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::InitHttpClientAsync()
{
  try {
    http_client_driver_->init_client(sensor_configuration_->sensor_ip, 80);
  } catch (const std::exception & ex) {
//    Status status = Status::HTTP_CONNECTION_ERROR;
    VelodyneStatus status = Status::HTTP_CONNECTION_ERROR;
//    std::cerr << status << sensor_configuration_->sensor_ip << ","
//              << 80 << std::endl;
    return status;
  }
  return Status::OK;
}

/*
std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> VelodyneHwInterface::GetHttpClientDriverOnce(std::shared_ptr<boost::asio::io_context> ctx){
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd = std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>(new ::drivers::tcp_driver::HttpClientDriver(ctx));
  try {
    hcd->init_client(sensor_configuration_->sensor_ip, 80);
  } catch (const std::exception & ex) {
    Status status = Status::HTTP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << 80 << std::endl;
    return hcd;
  }
  return hcd;
}

std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> VelodyneHwInterface::GetHttpClientDriverOnce(){
  return GetHttpClientDriverOnce(std::make_shared<boost::asio::io_context>());
}
*/

VelodyneStatus VelodyneHwInterface::GetHttpClientDriverOnce(std::shared_ptr<boost::asio::io_context> ctx, std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>& hcd){
  hcd = std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>(new ::drivers::tcp_driver::HttpClientDriver(ctx));
  try {
//    std::cout << sensor_configuration_->sensor_ip << std::endl;
    hcd->init_client(sensor_configuration_->sensor_ip, 80);
  } catch (const std::exception & ex) {
    Status status = Status::HTTP_CONNECTION_ERROR;
    std::cerr << status << sensor_configuration_->sensor_ip << ","
              << 80 << std::endl;
    return Status::HTTP_CONNECTION_ERROR;
  }
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::GetHttpClientDriverOnce(std::unique_ptr<::drivers::tcp_driver::HttpClientDriver>& hcd){
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd_tmp;
  auto st = GetHttpClientDriverOnce(std::make_shared<boost::asio::io_context>(), hcd_tmp);
  hcd = std::move(hcd_tmp);
  return st;
}

void VelodyneHwInterface::str_cb(const std::string &str)
{
  std::cout << "VelodyneHwInterface::str_cb: " << str <<std::endl;
//  RCLCPP_INFO(get_logger(), "VelodyneHwInterface::str_cb: %s", str.c_str());
}

boost::property_tree::ptree VelodyneHwInterface::ParseJson(const std::string &str)
{
//  std::cout << "VelodyneHwInterface::parse_json: " << str <<std::endl;
//  std::cout << "VelodyneHwInterface::parse_json" << std::endl;
  boost::property_tree::ptree tree;
  try {
      std::istrstream is(str.c_str());
      boost::property_tree::read_json(is, tree);

    /*
      std::string target_key = "config.returns";
      std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << tree.get<std::string>(target_key) << std::endl;
      target_key = "config.rpm";
      std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << tree.get<std::string>(target_key) << std::endl;
      target_key = "config.fov.start";
      std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << tree.get<std::string>(target_key) << std::endl;
      target_key = "config.fov.end";
      std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << tree.get<std::string>(target_key) << std::endl;
    */
  } catch (boost::property_tree::json_parser_error &e) {
      std::cerr << e.what() << std::endl;
  }
  return tree;
}

VelodyneStatus VelodyneHwInterface::CheckAndSetConfig(std::shared_ptr<VelodyneSensorConfiguration> sensor_configuration, boost::property_tree::ptree tree)
{
  std::string target_key = "config.returns";
  auto current_return_mode_str = tree.get<std::string>(target_key);
  auto current_return_mode = nebula::drivers::ReturnModeFromStringVelodyne(tree.get<std::string>(target_key));
  if(sensor_configuration->return_mode != current_return_mode)
  {
    SetReturnTypeAsync(sensor_configuration->return_mode);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_return_mode_str << std::endl;
    std::cout << "current_return_mode: " << current_return_mode << std::endl;
    std::cout << "sensor_configuration->return_mode: " << sensor_configuration->return_mode << std::endl;
  }

  target_key = "config.rpm";
//  auto current_rotation_speed = tree.get<std::string>(target_key);
  auto current_rotation_speed = tree.get<u_int16_t>(target_key);
  if(sensor_configuration->rotation_speed != current_rotation_speed)
  {
    SetRpmAsync(sensor_configuration->rotation_speed);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_rotation_speed << std::endl;
    std::cout << "sensor_configuration->rotation_speed: " << sensor_configuration->rotation_speed << std::endl;
  }

  target_key = "config.fov.start";
  auto current_cloud_min_angle = tree.get<std::uint16_t>(target_key);
  if(sensor_configuration->cloud_min_angle != current_cloud_min_angle)
  {
    SetFovStartAsync(sensor_configuration->cloud_min_angle);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_cloud_min_angle << std::endl;
    std::cout << "sensor_configuration->cloud_min_angle: " << sensor_configuration->cloud_min_angle << std::endl;
  }

  target_key = "config.fov.end";
  auto current_cloud_max_angle = tree.get<std::uint16_t>(target_key);
  if(sensor_configuration->cloud_max_angle != current_cloud_max_angle)
  {
    SetFovEndAsync(sensor_configuration->cloud_max_angle);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_cloud_max_angle << std::endl;
    std::cout << "sensor_configuration->cloud_max_angle: " << sensor_configuration->cloud_max_angle << std::endl;
  }

  target_key = "config.host.addr";
  auto current_host_addr = tree.get<std::string>(target_key);
  if(sensor_configuration->host_ip != current_host_addr)
  {
    SetHostAddrAsync(sensor_configuration->host_ip);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_addr << std::endl;
    std::cout << "sensor_configuration->host_ip: " << sensor_configuration->host_ip << std::endl;
  }

  target_key = "config.host.dport";
  auto current_host_dport = tree.get<std::uint16_t>(target_key);
  if(sensor_configuration->data_port != current_host_dport)
  {
    SetHostDportAsync(sensor_configuration->data_port);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_dport << std::endl;
    std::cout << "sensor_configuration->data_port: " << sensor_configuration->data_port << std::endl;
  }

  target_key = "config.host.tport";
  auto current_host_tport = tree.get<std::uint16_t>(target_key);
  if(sensor_configuration->gnss_port != current_host_tport)
  {
    SetHostTportAsync(sensor_configuration->gnss_port);
    std::cout << "VelodyneHwInterface::parse_json(" << target_key << "): " << current_host_tport << std::endl;
    std::cout << "sensor_configuration->gnss_port: " << sensor_configuration->gnss_port << std::endl;
  }

  return Status::WAITING_FOR_SENSOR_RESPONSE;
}


//sync

std::string VelodyneHwInterface::GetStatus()
{
  auto rt = http_client_driver_->get(
    TARGET_STATUS);
  http_client_driver_->client()->close();
//  str_cb(rt);
//  return Status::OK;
  return rt;
}

std::string VelodyneHwInterface::GetDiag()
{
  /*
  auto rt = http_client_driver_->get(
    TARGET_DIAG);
  http_client_driver_->client()->close();
//  str_cb(rt);
//  return Status::OK;
  return rt;
  */

  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  std::cout << "GetHttpClientDriverOnce" << std::endl;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return "";
  }
  auto rt = hcd->get(
    TARGET_DIAG);
  std::cout << "read_response: " << rt << std::endl;
//  hcd->client()->close();
  return rt;

}

std::string VelodyneHwInterface::GetSnapshot()
{
  auto rt = http_client_driver_->get(
    TARGET_SNAPSHOT);
  http_client_driver_->client()->close();
//  str_cb(rt);
//  ParseJson(rt);
//  return Status::OK;
  return rt;
}

VelodyneStatus VelodyneHwInterface::SetRpm(uint16_t rpm)
{
  if (rpm < 300 || 1200 < rpm || rpm % 60 != 0){
//    std::cerr << "Invalid integer value(range from 300 to 1200, in increments of 60): " << rpm << std::endl;
    return VelodyneStatus::INVALID_RPM_ERROR;
  }
  auto rt = http_client_driver_->post(
    TARGET_SETTING,
    (boost::format("rpm=%d") % rpm).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetFovStart(uint16_t fov_start)
{
  if (359 < fov_start){
//    std::cerr << "Invalid integer value(0 to 359): " << fov_start << std::endl;
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = http_client_driver_->post(
    TARGET_FOV,
    (boost::format("start=%d") % fov_start).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetFovEnd(uint16_t fov_end)
{
  if (359 < fov_end){
//    std::cerr << "Invalid integer value(0 to 359): " << fov_end << std::endl;
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
  auto rt = http_client_driver_->post(
    TARGET_FOV,
    (boost::format("end=%d") % fov_end).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetReturnType(nebula::drivers::ReturnMode return_mode)
{
  std::string body_str = "";
  switch(return_mode){
    case nebula::drivers::ReturnMode::SINGLE_STRONGEST:
      body_str = "returns=Strongest";
      break;
    case nebula::drivers::ReturnMode::SINGLE_LAST:
      body_str = "returns=Last";
      break;
    case nebula::drivers::ReturnMode::DUAL_ONLY:
      body_str = "returns=Dual";
      break;
    default:
//      std::cerr << "Invalid ReturnMode: " << return_mode << std::endl;
      return VelodyneStatus::INVALID_RETURN_MODE_ERROR;
  }
  auto rt = http_client_driver_->post(
    TARGET_SETTING,
    body_str
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SaveConfig()
{
  std::string body_str = "submit";
  auto rt = http_client_driver_->post(
    TARGET_SAVE,
    body_str
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::ResetSystem()
{
  std::string body_str = "reset_system";
  auto rt = http_client_driver_->post(
    TARGET_RESET,
    body_str
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::LaserOn()
{
  std::string body_str = "laser=on";
  auto rt = http_client_driver_->post(
    TARGET_SETTING,
    body_str
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::LaserOff()
{
  std::string body_str = "laser=off";
  auto rt = http_client_driver_->post(
    TARGET_SETTING,
    body_str
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::LaserOnOff(bool on)
{
  std::string body_str = (boost::format("laser=%s") % (on ? "on" : "off")).str();
  auto rt = http_client_driver_->post(
    TARGET_SETTING,
    body_str
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetHostAddr(std::string addr)
{
  auto rt = http_client_driver_->post(
    TARGET_HOST,
    (boost::format("addr=%s") % addr).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetHostDport(uint16_t dport)
{
  auto rt = http_client_driver_->post(
    TARGET_HOST,
    (boost::format("dport=%d") % dport).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetHostTport(uint16_t tport)
{
  auto rt = http_client_driver_->post(
    TARGET_HOST,
    (boost::format("tport=%d") % tport).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetAddr(std::string addr)
{
  auto rt = http_client_driver_->post(
    TARGET_NET,
    (boost::format("addr=%s") % addr).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetMask(std::string mask)
{
  auto rt = http_client_driver_->post(
    TARGET_NET,
    (boost::format("mask=%s") % mask).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetGateway(std::string gateway)
{
  auto rt = http_client_driver_->post(
    TARGET_NET,
    (boost::format("gateway=%s") % gateway).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

VelodyneStatus VelodyneHwInterface::SetNetDhcp(bool use_dhcp)
{
  auto rt = http_client_driver_->post(
    TARGET_NET,
    (boost::format("dhcp=%s") % (use_dhcp ? "on" : "off")).str()
    );
  http_client_driver_->client()->close();
  str_cb(rt);
  return Status::OK;
}

//async

VelodyneStatus VelodyneHwInterface::GetStatusAsync(std::function<void(const std::string &str)> str_callback)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

//  RCLCPP_INFO(get_logger(), "VelodyneHwInterface::GetStatus");
//  http_client_driver_->asyncGet(
  /*
  hcd->asyncGet(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_STATUS);
    */

  hcd->asyncGet(
    str_callback,
    TARGET_STATUS);
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::GetStatusAsync()
{
  return GetStatusAsync(
    [this](const std::string &str)
    {
      str_cb(str);
    });
}

VelodyneStatus VelodyneHwInterface::GetDiagAsync(std::function<void(const std::string &str)> str_callback)
{
//  std::cout << "make io_context" << std::endl;
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto ctx = std::make_shared<boost::asio::io_context>(3);
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
//  std::cout << "GetHttpClientDriverOnce" << std::endl;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

//  RCLCPP_INFO(get_logger(), "VelodyneHwInterface::GetDiag");
//  http_client_driver_->asyncGet(
  /*
  hcd->asyncGet(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_DIAG);
    */
//  std::cout << "hcd->asyncGet" << std::endl;
  hcd->asyncGet(
    str_callback,
    TARGET_DIAG);
//  std::cout << "ctx->run()" << std::endl;
  ctx->run();
//  ctx->poll();
//  std::cout << "return Status::WAITING_FOR_SENSOR_RESPONSE" << std::endl;
//  ctx.reset();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}


VelodyneStatus VelodyneHwInterface::GetDiagAsync()
{
  return GetDiagAsync(
    [this](const std::string &str)
    {
      str_cb(str);
    });
}


VelodyneStatus VelodyneHwInterface::GetSnapshotAsync(std::function<void(const std::string &str)> str_callback)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }
  /*
  hcd->asyncGet(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SNAPSHOT);
    */
  hcd->asyncGet(
    str_callback,
    TARGET_SNAPSHOT);
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::GetSnapshotAsync()
{
  return GetSnapshotAsync(
    [this](const std::string &str)
    {
//      str_cb(str);
      ParseJson(str);
    });
}

VelodyneStatus VelodyneHwInterface::CheckAndSetConfigBySnapshotAsync()
{
  return GetSnapshotAsync(
    [this](const std::string &str)
    {
      auto tree = ParseJson(str);
      CheckAndSetConfig(std::static_pointer_cast<VelodyneSensorConfiguration>(sensor_configuration_), tree);
    });
}

VelodyneStatus VelodyneHwInterface::SetRpmAsync(uint16_t rpm)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

//  RCLCPP_INFO(get_logger(), "VelodyneHwInterface::SetRpm: %d", rpm);
  if (rpm < 300 || 1200 < rpm || rpm % 60 != 0){
//    std::cerr << "Invalid integer value(range from 300 to 1200, in increments of 60): " << rpm << std::endl;
//    return Status::SENSOR_CONFIG_ERROR;
    return VelodyneStatus::INVALID_RPM_ERROR;
  }
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SETTING,
    (boost::format("rpm=%d") % rpm).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetFovStartAsync(uint16_t fov_start)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  if (359 < fov_start){
//    std::cerr << "Invalid integer value(0 to 359): " << fov_start << std::endl;
//    return Status::SENSOR_CONFIG_ERROR;
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_FOV,
    (boost::format("start=%d") % fov_start).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetFovEndAsync(uint16_t fov_end)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  if (359 < fov_end){
//    std::cerr << "Invalid integer value(0 to 359): " << fov_end << std::endl;
//    return Status::SENSOR_CONFIG_ERROR;
    return VelodyneStatus::INVALID_FOV_ERROR;
  }
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_FOV,
    (boost::format("end=%d") % fov_end).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetReturnTypeAsync(nebula::drivers::ReturnMode return_mode)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  std::string body_str = "";
  switch(return_mode){
    case nebula::drivers::ReturnMode::SINGLE_STRONGEST:
      body_str = "returns=Strongest";
      break;
    case nebula::drivers::ReturnMode::SINGLE_LAST:
      body_str = "returns=Last";
      break;
    case nebula::drivers::ReturnMode::DUAL_ONLY:
      body_str = "returns=Dual";
      break;
    default:
//      std::cerr << "Invalid ReturnMode: " << return_mode << std::endl;
//      return Status::SENSOR_CONFIG_ERROR;
      return VelodyneStatus::INVALID_RETURN_MODE_ERROR;
  }
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SETTING,
    body_str
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SaveConfigAsync()
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  std::string body_str = "submit";
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SAVE,
    body_str
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::ResetSystemAsync()
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  std::string body_str = "reset_system";
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_RESET,
    body_str
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::LaserOnAsync()
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  std::string body_str = "laser=on";
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SETTING,
    body_str
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::LaserOffAsync()
{
  auto ctx = std::make_shared<boost::asio::io_context>();
//  auto hcd = GetHttpClientDriverOnce(ctx);
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  std::string body_str = "laser=off";
//  http_client_driver_->asyncPost(
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SETTING,
    body_str
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::LaserOnOffAsync(bool on)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  std::string body_str = (boost::format("laser=%s") % (on ? "on" : "off")).str();
  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_SETTING,
    body_str
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetHostAddrAsync(std::string addr)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_HOST,
    (boost::format("addr=%s") % addr).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetHostDportAsync(uint16_t dport)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_HOST,
    (boost::format("dport=%d") % dport).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetHostTportAsync(uint16_t tport)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_HOST,
    (boost::format("tport=%d") % tport).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetNetAddrAsync(std::string addr)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_NET,
    (boost::format("addr=%s") % addr).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetNetMaskAsync(std::string mask)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_NET,
    (boost::format("mask=%s") % mask).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetNetGatewayAsync(std::string gateway)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_NET,
    (boost::format("gateway=%s") % gateway).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

VelodyneStatus VelodyneHwInterface::SetNetDhcpAsync(bool use_dhcp)
{
  auto ctx = std::make_shared<boost::asio::io_context>();
  std::unique_ptr<::drivers::tcp_driver::HttpClientDriver> hcd;
  auto st = GetHttpClientDriverOnce(ctx, hcd);
  if(st != Status::OK){
    return st;
  }

  hcd->asyncPost(
    [this](const std::string &str)
    {
      str_cb(str);
    },
    TARGET_NET,
    (boost::format("dhcp=%s") % (use_dhcp ? "on" : "off")).str()
    );
  ctx->run();
  return Status::WAITING_FOR_SENSOR_RESPONSE;
}

void VelodyneHwInterface::SetLogger(std::shared_ptr<rclcpp::Logger> logger)
{
  parent_node_logger = logger;
}

void VelodyneHwInterface::PrintInfo(std::string info)
{
  if(parent_node_logger)
  {
    RCLCPP_INFO_STREAM(*parent_node_logger, info);
  }
  else
  {
    std::cout << info << std::endl;
  }
}

void VelodyneHwInterface::PrintError(std::string error)
{
  if(parent_node_logger)
  {
    RCLCPP_ERROR_STREAM(*parent_node_logger, error);
  }
  else
  {
    std::cerr << error << std::endl;
  }
}

void VelodyneHwInterface::PrintDebug(std::string debug)
{
  if(parent_node_logger)
  {
    RCLCPP_DEBUG_STREAM(*parent_node_logger, debug);
  }
  else
  {
    std::cout << debug << std::endl;
  }
}


}  // namespace drivers
}  // namespace nebula