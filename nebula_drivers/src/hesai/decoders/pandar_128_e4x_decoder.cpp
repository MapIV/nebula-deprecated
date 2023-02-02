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



  last_phase_ = 0;
  has_scanned_ = false;

  scan_pc_.reset(new PointCloudXYZIRADT);
  overflow_pc_.reset(new PointCloudXYZIRADT);
}

bool Pandar128E4XDecoder::hasScanned() { return has_scanned_; }

drivers::PointCloudXYZIRADTPtr Pandar128E4XDecoder::get_pointcloud() { return scan_pc_; }

bool Pandar128E4XDecoder::parsePacket(const pandar_msgs::msg::PandarPacket & raw_packet)
{
  if (raw_packet.size != sizeof(PacketExtended)) {
    std::cerr << "Extended Packet not supported yet. Please change 'UDP Extra Bytes Type' to 'No Extra'";
    return false;
  }

  if (raw_packet.size != sizeof(Packet)) {
    // packet size mismatch !
    std::cerr << "Packet size mismatch. Expected: " << sizeof(Packet) << ", received:" << raw_packet.size;
    return false;
  }
  if (std::memcpy(&packet_, raw_packet.data.data(), sizeof(Packet)) != nullptr) {
    return true;
  }
  return false;
}

void Pandar128E4XDecoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (!parsePacket(pandar_packet)) {
    return;
  }
  if (packet_.tail.return_mode == SINGLE_FIRST_RETURN
      || packet_.tail.return_mode == SINGLE_STRONGEST_RETURN
      || packet_.tail.return_mode == SINGLE_LAST_RETURN)
  {

  }
}

drivers::PointXYZIRADT Pandar128E4XDecoder::build_point(
  size_t block_id, size_t unit_id, ReturnMode return_type)
{
  PointXYZIRADT point{};

  return point;
}

drivers::PointCloudXYZIRADTPtr Pandar128E4XDecoder::convert(size_t block_id)
{
  drivers::PointCloudXYZIRADTPtr block_pc(new PointCloudXYZIRADT);


  return block_pc;
}

drivers::PointCloudXYZIRADTPtr Pandar128E4XDecoder::convert_dual(size_t block_id)
{

}

}  // namespace pandar_40
}  // namespace drivers
}  // namespace nebula