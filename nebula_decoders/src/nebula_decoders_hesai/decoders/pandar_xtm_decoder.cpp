#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xtm_decoder.hpp"

#include "nebula_decoders/nebula_decoders_hesai/decoders/pandar_xtm.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_xtm
{
PandarXTMDecoder::PandarXTMDecoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }
  /////////////////

  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
  }

  m_sin_elevation_map_.resize(LASER_COUNT);
  m_cos_elevation_map_.resize(LASER_COUNT);
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    m_sin_elevation_map_[laser] = sinf(deg2rad(elev_angle_[laser]));
    m_cos_elevation_map_[laser] = cosf(deg2rad(elev_angle_[laser]));
  }
  m_sin_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  for (int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    m_sin_azimuth_map_[i] = sinf(i * M_PI / 18000);
    m_cos_azimuth_map_[i] = cosf(i * M_PI / 18000);
  }

  scan_phase_ = static_cast<uint16_t>(sensor_configuration_->scan_phase * 100.0f);
  dual_return_distance_threshold_ = sensor_configuration_->dual_return_distance_threshold;

  last_phase_ = 0;
  has_scanned_ = false;
  first_timestamp_tmp = std::numeric_limits<double>::max();
  first_timestamp_ = first_timestamp_tmp;

  scan_pc_.reset(new NebulaPointCloud);
  overflow_pc_.reset(new NebulaPointCloud);
}

bool PandarXTMDecoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> PandarXTMDecoder::get_pointcloud()
{
  return std::make_tuple(scan_pc_, first_timestamp_);
}

void PandarXTMDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }

  if (has_scanned_) {
    scan_pc_ = overflow_pc_;
    first_timestamp_ = first_timestamp_tmp;
    first_timestamp_tmp = std::numeric_limits<double>::max();
    overflow_pc_.reset(new NebulaPointCloud);
    has_scanned_ = false;
  }

  /*
  bool dual_return =
    (packet_.return_mode != FIRST_RETURN && packet_.return_mode != STRONGEST_RETURN &&
     packet_.return_mode != LAST_RETURN);
  auto step = dual_return ? 2 : 1;

  for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id += step) {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase =
      (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
    if (current_phase > last_phase_ && !has_scanned_) {
      *scan_pc_ += *block_pc;
    } else {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
  */
  for (int block_id = 0; block_id < packet_.header.chBlockNumber; ++block_id) {
    int azimuthGap = 0;      /* To do */
    double timestampGap = 0; /* To do */
    if (last_azimuth_ > packet_.blocks[block_id].azimuth) {
      azimuthGap = static_cast<int>(packet_.blocks[block_id].azimuth) +
                   (36000 - static_cast<int>(last_azimuth_));
    } else {
      azimuthGap =
        static_cast<int>(packet_.blocks[block_id].azimuth) - static_cast<int>(last_azimuth_);
    }
    timestampGap = packet_.usec - last_timestamp_ + 0.001;
    auto block_pc = convert(block_id);
    if (
      last_azimuth_ != packet_.blocks[block_id].azimuth &&
      (azimuthGap / timestampGap) < 36000 * 100) {
      /* for all the blocks */
      if (
        (last_azimuth_ > packet_.blocks[block_id].azimuth &&
         start_angle_ <= packet_.blocks[block_id].azimuth) ||
        (last_azimuth_ < start_angle_ && start_angle_ <= packet_.blocks[block_id].azimuth)) {
        *overflow_pc_ += *block_pc;
        has_scanned_ = true;
      }
    } else {
      *scan_pc_ += *block_pc;
      //printf("last_azimuth_:%d pkt.blocks[block_id].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[block_id].azimuth, azimuthGap);
    }
    //    CalcXTPointXYZIT(block_id, packet_.header.chLaserNumber, scan_pc_);
    last_azimuth_ = packet_.blocks[block_id].azimuth;
    last_timestamp_ = packet_.usec;
  }
}

#if defined(ROS_DISTRO_FOXY) || defined(ROS_DISTRO_GALACTIC)
void PandarXTMDecoder::CalcXTPointXYZIT(
  int blockid, char chLaserNumber, boost::shared_ptr<pcl::PointCloud<NebulaPoint>> cld)
{
#else
void PandarXTMDecoder::CalcXTPointXYZIT(
  int blockid, char chLaserNumber, std::shared_ptr<pcl::PointCloud<NebulaPoint>> cld)
{
#endif
  Block * block = &packet_.blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    Unit & unit = block->units[i];
    NebulaPoint point{};

    /* skip wrong points */
    //    if (unit.distance <= 0.1 || unit.distance > 200.0) {
    if (unit.distance <= 0.1 || unit.distance > 300.0) {
      //      std::cout << "unit.distance <= 0.1 || unit.distance > 300.0" << std::endl;
      continue;
    }

    int azimuth = static_cast<int>(azimuth_offset_[i] * 100 + block->azimuth);
    if (azimuth < 0) azimuth += 36000;
    if (azimuth >= 36000) azimuth -= 36000;

    {
      float xyDistance = unit.distance * m_cos_elevation_map_[i];
      point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
      point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
      point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
    }

    point.intensity = unit.intensity;

    double unix_second = static_cast<double>(timegm(&packet_.t));  // sensor-time (ppt/gps)
    if (unix_second < first_timestamp_tmp) {
      first_timestamp_tmp = unix_second;
    }
    point.time_stamp = (static_cast<double>(packet_.usec)) / 1000000.0;
    point.time_stamp +=
      (static_cast<double>(blockXTMOffsetSingle[i] + laserXTMOffset[i]) / 1000000.0f);

    if (packet_.return_mode == TRIPLE_RETURN) {
      point.time_stamp =
        point.time_stamp +
        (static_cast<double>(blockXTMOffsetTriple[blockid] + laserXTMOffset[i]) / 1000000.0f);
    } else if (
      packet_.return_mode == DUAL_RETURN || packet_.return_mode == DUAL_RETURN_B ||
      packet_.return_mode == DUAL_RETURN_C) {
      point.time_stamp =
        point.time_stamp +
        (static_cast<double>(blockXTMOffsetDual[blockid] + laserXTMOffset[i]) / 1000000.0f);
    } else {
      point.time_stamp =
        point.time_stamp +
        (static_cast<double>(blockXTMOffsetSingle[blockid] + laserXTMOffset[i]) / 1000000.0f);
    }

    //    point.return_type = packet_.return_mode;
    switch (packet_.return_mode) {
      case DUAL_RETURN:  // Dual Return (Last, Strongest)
        if (i % 2 == 0) {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::LAST);
        } else {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
        }
        break;

      case DUAL_RETURN_B:  // Dual Return (Last, First)
        if (i % 2 == 0) {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::LAST);
        } else {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::FIRST);
        }
        break;

      case DUAL_RETURN_C:  // Dual Return (First, Strongest)
        if (i % 2 == 0) {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::FIRST);
        } else {
          point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::STRONGEST);
        }
        break;

      default:
        point.return_type = static_cast<uint8_t>(nebula::drivers::ReturnType::UNKNOWN);
        break;
    }
    point.channel = i;
    cld->points.emplace_back(point);
  }
}

drivers::NebulaPointCloudPtr PandarXTMDecoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);
  CalcXTPointXYZIT(block_id, packet_.header.chLaserNumber, block_pc);

  return block_pc;
}

drivers::NebulaPointCloudPtr PandarXTMDecoder::convert_dual(size_t block_id)
{
  return convert(block_id);
}

bool PandarXTMDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE) {
    std::cout << "pandar_packet.size != PACKET_SIZE" << std::endl;
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

  size_t index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    // Error Start of Packet!
    std::cout << "Error Start of Packet!" << std::endl;
    return false;
  }

  for (int8_t block = 0; block < packet_.header.chBlockNumber; block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_HEADER_AZIMUTH;

    for (int unit = 0; unit < packet_.header.chLaserNumber; unit++) {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
        (static_cast<float>(unRange * packet_.header.chDisUnit)) / 1000.f;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
      index += UNIT_SIZE;
    }
  }

  index += RESERVED_SIZE;  // skip reserved bytes
  packet_.return_mode = buf[index] & 0xff;

  index += RETURN_SIZE;
  index += ENGINE_VELOCITY;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;
  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  index += UTC_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;
  index += FACTORY_SIZE;

  return true;
}
}  // namespace pandar_xtm
}  // namespace drivers
}  // namespace nebula