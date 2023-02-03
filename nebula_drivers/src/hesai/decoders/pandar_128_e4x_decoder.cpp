#include <cmath>
#include <utility>

#include "hesai/decoders/pandar_128_e4x_decoder.hpp"
#include "hesai/decoders/pandar_128_e4x.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_128_e4x
{
Pandar128E4XDecoder::Pandar128E4XDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

//  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
//    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
//    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
//  }
  elev_angle_ = {
      14.436f,  13.535f,  13.08f,   12.624f,  12.163f,  11.702f,  11.237f,
      10.771f,  10.301f,  9.83f,    9.355f,   8.88f,    8.401f,   7.921f,
      7.437f,   6.954f,   6.467f,   5.98f,    5.487f,   4.997f,   4.501f,
      4.009f,   3.509f,   3.014f,   2.512f,   2.014f,   1.885f,   1.761f,
      1.637f,   1.511f,   1.386f,   1.258f,   1.13f,    1.009f,   0.88f,
      0.756f,   0.63f,    0.505f,   0.379f,   0.251f,   0.124f,   0.0f,
      -0.129f,  -0.254f,  -0.38f,   -0.506f,  -0.632f,  -0.76f,   -0.887f,
      -1.012f,  -1.141f,  -1.266f,  -1.393f,  -1.519f,  -1.646f,  -1.773f,
      -1.901f,  -2.027f,  -2.155f,  -2.282f,  -2.409f,  -2.535f,  -2.662f,
      -2.789f,  -2.916f,  -3.044f,  -3.172f,  -3.299f,  -3.425f,  -3.552f,
      -3.680f,  -3.806f,  -3.933f,  -4.062f,  -4.190f,  -4.318f,  -4.444f,
      -4.571f,  -4.698f,  -4.824f,  -4.951f,  -5.081f,  -5.209f,  -5.336f,
      -5.463f,  -5.589f,  -5.717f,  -5.843f,  -5.968f,  -6.099f,  -6.607f,
      -7.118f,  -7.624f,  -8.135f,  -8.64f,   -9.149f,  -9.652f,  -10.16f,
      -10.664f, -11.17f,  -11.67f,  -12.174f, -12.672f, -13.173f, -13.668f,
      -14.166f, -14.658f, -15.154f, -15.643f, -16.135f, -16.62f,  -17.108f,
      -17.59f,  -18.073f, -18.548f, -19.031f, -19.501f, -19.981f, -20.445f,
      -20.92f,  -21.379f, -21.85f,  -22.304f, -22.77f,  -23.219f, -23.68f,
      -24.123f, -25.016f,
  };
  for (auto &angle:elev_angle_){
    angle = deg2rad(angle);
  }
  azimuth_offset_ = {
      3.257f,  3.263f, -1.083f, 3.268f, -1.086f, 3.273f,  -1.089f, 3.278f,
      -1.092f, 3.283f, -1.094f, 3.288f, -1.097f, 3.291f,  -1.1f,   1.1f,
      -1.102f, 1.1f,   -3.306f, 1.102f, -3.311f, 1.103f,  -3.318f, 1.105f,
      -3.324f, 1.106f, 7.72f,   5.535f, 3.325f,  -3.33f,  -1.114f, -5.538f,
      -7.726f, 1.108f, 7.731f,  5.543f, 3.329f,  -3.336f, -1.116f, -5.547f,
      -7.738f, 1.108f, 7.743f,  5.551f, 3.335f,  -3.342f, -1.119f, -5.555f,
      -7.75f,  1.11f,  7.757f,  5.56f,  3.34f,   -3.347f, -1.121f, -5.564f,
      -7.762f, 1.111f, 7.768f,  5.569f, 3.345f,  -3.353f, -1.123f, -5.573f,
      -7.775f, 1.113f, 7.780f,  5.578f, 3.351f,  -3.358f, -1.125f, -5.582f,
      -7.787f, 1.115f, 7.792f,  5.586f, 3.356f,  -3.363f, -1.126f, -5.591f,
      -7.799f, 1.117f, 7.804f,  5.595f, 3.36f,   -3.369f, -1.128f, -5.599f,
      -7.811f, 1.119f, -3.374f, 1.12f,  -3.379f, 1.122f,  -3.383f, 3.381f,
      -3.388f, 3.386f, -1.135f, 3.39f,  -1.137f, 3.395f,  -1.138f, 3.401f,
      -1.139f, 3.406f, -1.14f,  3.41f,  -1.141f, 3.416f,  -1.142f, 1.14f,
      -1.143f, 1.143f, -3.426f, 1.146f, -3.429f, 1.147f,  -3.433f, 1.15f,
      -3.436f, 1.152f, -3.44f,  1.154f, -3.443f, 1.157f,  -3.446f, -3.449f,
  };
  for (auto &angle:azimuth_offset_){
    angle = deg2rad(angle);
  }

  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new PointCloudXYZIRADT);
  scan_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
  overflow_pc_.reset(new PointCloudXYZIRADT);
  overflow_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
}

bool Pandar128E4XDecoder::hasScanned() { return has_scanned_; }

drivers::PointCloudXYZIRADTPtr Pandar128E4XDecoder::get_pointcloud() { return scan_pc_; }

bool Pandar128E4XDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & raw_packet)
{
  if (raw_packet.size != sizeof(Packet)) {
    std::cerr << "Packet size mismatch:" << raw_packet.size
              << "| Expected:" << sizeof(Packet) << std::endl;
    return false;
  }
  if (std::memcpy(&packet_, raw_packet.data.data(), sizeof(Packet))) {
    return true;
  }
  std::cerr << "Invalid SOF " << std::hex << packet_.header.SOP << " Packet" << std::endl;
  return false;
}

void Pandar128E4XDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }
  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    overflow_pc_.reset(new PointCloudXYZIRADT);
    overflow_pc_->reserve(LASER_COUNT*MAX_AZIMUTH_STEPS);
    has_scanned_ = false;
  }

  bool dual_return = false;
  if (packet_.tail.return_mode == DUAL_LAST_STRONGEST_RETURN
      || packet_.tail.return_mode == DUAL_LAST_FIRST_RETURN
      || packet_.tail.return_mode == DUAL_FIRST_STRONGEST_RETURN) {
    dual_return = true;
  }

  auto block_pc = convert();
  int current_phase =
      (static_cast<int>(packet_.body.azimuth_1) - scan_phase_ + 36000) % 36000;
  if (current_phase > last_phase_ && !has_scanned_) {
    *scan_pc_ += *block_pc;
  } else {
    *overflow_pc_ += *block_pc;
    has_scanned_ = true;
  }
  last_phase_ = current_phase;

}

drivers::PointXYZIRADT Pandar128E4XDecoder::build_point(const Block& block,
                                                        const size_t& laser_id,
                                                        const uint16_t& azimuth,
                                                        const double& unix_second)
{

  PointXYZIRADT point{};

  double xyDistance = block.distance * cos(elev_angle_[laser_id]);

  point.x = static_cast<float>(
      xyDistance *
      sin(deg2rad(azimuth_offset_[laser_id] + (static_cast<double>(azimuth)) / 100.0)));
  point.y = static_cast<float>(
      xyDistance *
      cos(deg2rad(azimuth_offset_[laser_id] + (static_cast<double>(azimuth)) / 100.0)));
  point.z = static_cast<float>(block.distance * sin(elev_angle_[laser_id]));

  point.intensity = block.reflectivity;
  point.distance = block.distance;
  point.ring = laser_id;
  point.azimuth = azimuth + std::round(azimuth_offset_[laser_id] * 100.0f);
  point.return_type = 0; // TODO
  point.time_stamp = unix_second + packet_.tail.timestamp_us * 1e-6;

  return point;
}

drivers::PointCloudXYZIRADTPtr Pandar128E4XDecoder::convert()
{
  drivers::PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);
  block_pc->reserve(LASER_COUNT*2);
  struct tm t = {};
  t.tm_year = packet_.tail.date_time.year;
  t.tm_mon = packet_.tail.date_time.month - 1;
  t.tm_mday = packet_.tail.date_time.day;
  t.tm_hour = packet_.tail.date_time.hour;
  t.tm_min = packet_.tail.date_time.minute;
  t.tm_sec = packet_.tail.date_time.second;
  t.tm_isdst = 0;
  auto unix_second = static_cast<double>(timegm(&t));

  for(size_t i= 0; i < LASER_COUNT; i++) {
    auto block1_pt = build_point(packet_.body.block_01[i],
                                 i,
                                 packet_.body.azimuth_1,
                                 unix_second);

    auto block2_pt = build_point(packet_.body.block_02[i],
                                 i,
                                 packet_.body.azimuth_2,
                                 unix_second);
    if (block1_pt.distance >= MIN_RANGE && block1_pt.distance <= MAX_RANGE) {
      block_pc->points.emplace_back(block1_pt);
    }
    if (block2_pt.distance >= MIN_RANGE && block2_pt.distance <= MAX_RANGE) {
      block_pc->points.emplace_back(block2_pt);
    }
  }

  return block_pc;
}

drivers::PointCloudXYZIRADTPtr Pandar128E4XDecoder::convert_dual()
{
  drivers::PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);
  struct tm t = {};
  t.tm_year = packet_.tail.date_time.year;
  t.tm_mon = packet_.tail.date_time.month - 1;
  t.tm_mday = packet_.tail.date_time.day;
  t.tm_hour = packet_.tail.date_time.hour;
  t.tm_min = packet_.tail.date_time.minute;
  t.tm_sec = packet_.tail.date_time.second;
  t.tm_isdst = 0;

  for(size_t i= 0; i < LASER_COUNT; i++) {
    block_pc->points.emplace_back(
        build_point(packet_.body.block_01[i],
                    i,
                    packet_.body.azimuth_1,
                    static_cast<double>(timegm(&t)))
    );
    // TODO check the second block and compare with first
  }

  return block_pc;
}

}  // namespace pandar_40
}  // namespace drivers
}  // namespace nebula