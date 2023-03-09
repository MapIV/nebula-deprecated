#include "hesai/decoders/pandar_qt_128_decoder.hpp"

#include <cmath>

#include "hesai/decoders/pandar_qt_128.hpp"

namespace nebula
{
namespace drivers
{
namespace pandar_qt_128
{

  std::string PandarQT128_TL1 = R"(33,27.656
34,53
35,2.312
36,78.344
37,81.512
38,5.48
39,56.168
40,30.824
41,33.992
42,59.336
43,8.648
44,84.68
45,87.848
46,11.816
47,62.504
48,37.16
49,40.328
50,65.672
51,14.984
52,91.016
53,94.184
54,18.152
55,68.84
56,43.496
57,46.664
58,72.008
59,21.32
60,97.352
61,100.52
62,24.488
63,75.176
64,49.832
65,1.456
66,77.488
67,26.8
68,52.144
69,55.312
70,29.968
71,80.656
72,4.624
73,7.792
74,83.824
75,33.136
76,58.48
77,61.648
78,36.304
79,86.992
80,10.96
81,14.128
82,90.16
83,39.472
84,64.816
85,67.984
86,42.64
87,93.328
88,17.296
89,20.464
90,96.496
91,45.808
92,71.152
93,74.32
94,48.976
95,99.664
96,23.632
97,25.944
98,51.288
99,0.6
100,76.632
101,79.8
102,3.768
103,54.456
104,29.112
105,32.28
106,57.624
107,6.936
108,82.968
109,86.136
110,10.104
111,60.792
112,35.448
113,38.616
114,63.96
115,13.272
116,89.304
117,92.472
118,16.44
119,67.128
120,41.784
121,44.952
122,70.296
123,19.608
124,95.64
125,98.808
126,22.776
127,73.464
128,48.12)";
  std::string PandarQT128_TL2 = R"(1,2.312
2,78.344
3,27.656
4,53
5,56.168
6,30.824
7,81.512
8,5.48
9,8.648
10,84.68
11,33.992
12,59.336
13,62.504
14,37.16
15,87.848
16,11.816
17,14.984
18,91.016
19,40.328
20,65.672
21,68.84
22,43.496
23,94.184
24,18.152
25,21.32
26,97.352
27,46.664
28,72.008
29,75.176
30,49.832
31,100.52
32,24.488
65,0.6
66,76.632
67,25.944
68,51.288
69,54.456
70,29.112
71,79.8
72,3.768
73,6.936
74,82.968
75,32.28
76,57.624
77,60.792
78,35.448
79,86.136
80,10.104
81,13.272
82,89.304
83,38.616
84,63.96
85,67.128
86,41.784
87,92.472
88,16.44
89,19.608
90,95.64
91,44.952
92,70.296
93,73.464
94,48.12
95,98.808
96,22.776
97,26.8
98,52.144
99,1.456
100,77.488
101,80.656
102,4.624
103,55.312
104,29.968
105,33.136
106,58.48
107,7.792
108,83.824
109,86.992
110,10.96
111,61.648
112,36.304
113,39.472
114,64.816
115,14.128
116,90.16
117,93.328
118,17.296
119,67.984
120,42.64
121,45.808
122,71.152
123,20.464
124,96.496
125,99.664
126,23.632
127,74.32
128,48.976)";

PandarQT128Decoder::PandarQT128Decoder(
  const std::shared_ptr<drivers::HesaiSensorConfiguration> & sensor_configuration,
  const std::shared_ptr<drivers::HesaiCalibrationConfiguration> & calibration_configuration)
{
  sensor_configuration_ = sensor_configuration;
  sensor_calibration_ = calibration_configuration;
/*
  firing_offset_ = {
    12.31,  14.37,  16.43,  18.49,  20.54,  22.6,   24.66,  26.71,  29.16,  31.22,  33.28,
    35.34,  37.39,  39.45,  41.5,   43.56,  46.61,  48.67,  50.73,  52.78,  54.84,  56.9,
    58.95,  61.01,  63.45,  65.52,  67.58,  69.63,  71.69,  73.74,  75.8,   77.86,  80.9,
    82.97,  85.02,  87.08,  89.14,  91.19,  93.25,  95.3,   97.75,  99.82,  101.87, 103.93,
    105.98, 108.04, 110.1,  112.15, 115.2,  117.26, 119.32, 121.38, 123.43, 125.49, 127.54,
    129.6,  132.05, 134.11, 136.17, 138.22, 140.28, 142.34, 144.39, 146.45,
  };
*/
  {
    std::string sbuf;
    std::stringstream ss(PandarQT128_TL1);
    while( std::getline(ss, sbuf, '\n') ) {
      int id;
      float val;
      sscanf(sbuf.c_str(), "%d,%f", &id, &val);
      firing_offset1_[id] = val;
      /*
      std::string id;
      std::string val;
      std::stringstream ss2(sbuf);
      std::getline(ss2, id, ",")
      std::getline(ss2, val, ",")
      v.push_back(buffer);
      */
    }
  }
  {
    std::string sbuf;
    std::stringstream ss(PandarQT128_TL2);
    while( std::getline(ss, sbuf, '\n') ) {
      int id;
      float val;
      sscanf(sbuf.c_str(), "%d,%f", &id, &val);
      firing_offset2_[id] = val;
    }
  }
  for (size_t block = 0; block < BLOCKS_PER_PACKET; ++block) {
    block_offset_single_[block] = 9.00f + 111.11f * static_cast<float>(block);
    block_offset_dual_[block] = 9.00f;
  }

  // TODO: add calibration data validation
  // if(calibration.elev_angle_map.size() != num_lasers_){
  //   // calibration data is not valid!
  // }
  for (size_t laser = 0; laser < LASER_COUNT; ++laser) {
    elev_angle_[laser] = calibration_configuration->elev_angle_map[laser];
    azimuth_offset_[laser] = calibration_configuration->azimuth_offset_map[laser];
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

bool PandarQT128Decoder::hasScanned() { return has_scanned_; }

std::tuple<drivers::NebulaPointCloudPtr, double> PandarQT128Decoder::get_pointcloud() { return std::make_tuple(scan_pc_, first_timestamp_); }

void PandarQT128Decoder::unpack(const pandar_msgs::msg::PandarPacket & pandar_packet)
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

//  std::cout << "packet_.return_mode = " << packet_.return_mode << std::endl;
  bool dual_return = (packet_.return_mode == DUAL_LAST_STRONGEST_RETURN || 
  packet_.return_mode == DUAL_FIRST_LAST_RETURN ||
  packet_.return_mode == DUAL_FIRST_STRONGEST_RETURN ||
  packet_.return_mode == DUAL_STRONGEST_2ndSTRONGEST_RETURN ||
  packet_.return_mode == DUAL_FIRST_SECOND_RETURN);
  auto step = dual_return ? 2 : 1;
//  std::cout << "dual_return = " << dual_return << std::endl;

  if (!dual_return) {
    if (
      (packet_.return_mode == FIRST_RETURN &&
       sensor_configuration_->return_mode != drivers::ReturnMode::FIRST) ||
      (packet_.return_mode == LAST_RETURN &&
       sensor_configuration_->return_mode != drivers::ReturnMode::LAST)) {
      //sensor config, driver mismatched
    }
  }

  for (size_t block_id = 0; block_id < BLOCKS_PER_PACKET; block_id += step) {
    auto block_pc = dual_return ? convert_dual(block_id) : convert(block_id);
    int current_phase =
      (static_cast<int>(packet_.blocks[block_id].azimuth) - scan_phase_ + 36000) % 36000;
//    std::cout << "current_phase = " << current_phase << std::endl;
//    std::cout << "last_phase_ = " << last_phase_ << std::endl;

    if (current_phase > last_phase_ && !has_scanned_) {
      *scan_pc_ += *block_pc;
    } else {
      *overflow_pc_ += *block_pc;
      has_scanned_ = true;
    }
    last_phase_ = current_phase;
  }
}

drivers::NebulaPoint PandarQT128Decoder::build_point(
  size_t block_id, size_t unit_id, uint8_t return_type)
{
  const auto & block = packet_.blocks[block_id];
  const auto & unit = block.units[unit_id];
  auto unix_second = static_cast<double>(timegm(&packet_.t));
  if(unix_second < first_timestamp_tmp){
    first_timestamp_tmp = unix_second;
  }
  bool dual_return = (packet_.return_mode == DUAL_LAST_STRONGEST_RETURN || 
  packet_.return_mode == DUAL_FIRST_LAST_RETURN ||
  packet_.return_mode == DUAL_FIRST_STRONGEST_RETURN ||
  packet_.return_mode == DUAL_STRONGEST_2ndSTRONGEST_RETURN ||
  packet_.return_mode == DUAL_FIRST_SECOND_RETURN);
  NebulaPoint point{};
//  std::cout << "dual_return = " << dual_return << std::endl;

  double xyDistance = unit.distance * cosf(deg2rad(elev_angle_[unit_id]));

  point.x = static_cast<float>(
    xyDistance *
    sinf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
  point.y = static_cast<float>(
    xyDistance *
    cosf(deg2rad(azimuth_offset_[unit_id] + (static_cast<double>(block.azimuth)) / 100.0)));
  point.z = static_cast<float>(unit.distance * sinf(deg2rad(elev_angle_[unit_id])));

  point.intensity = unit.intensity;
//  point.distance = unit.distance;
  point.channel = unit_id;
  point.azimuth = block.azimuth + std::round(azimuth_offset_[unit_id] * 100.0f);
  point.return_type = packet_.return_mode; // keep original value

  point.time_stamp = (static_cast<double>(packet_.usec)) / 1000000.0;
  if(0 < packet_.mode_flag){
    point.time_stamp +=
      dual_return
        ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset1_[unit_id]) / 1000000.0f)
        : (static_cast<double>(block_offset_single_[block_id] + firing_offset1_[unit_id]) /
          1000000.0f);
  }else{
    point.time_stamp +=
      dual_return
        ? (static_cast<double>(block_offset_dual_[block_id] + firing_offset2_[unit_id]) / 1000000.0f)
        : (static_cast<double>(block_offset_single_[block_id] + firing_offset2_[unit_id]) /
          1000000.0f);
  }

//  std::cout << "point.time_stamp = " << point.time_stamp << std::endl;

  return point;
}

drivers::NebulaPointCloudPtr PandarQT128Decoder::convert(size_t block_id)
{
  NebulaPointCloudPtr block_pc(new NebulaPointCloud);

  const auto & block = packet_.blocks[block_id];
  for (size_t unit_id = 0; unit_id < LASER_COUNT; ++unit_id) {
    const auto & unit = block.units[unit_id];
    // skip invalid points
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    block_pc->points.emplace_back(build_point(
      block_id, unit_id, static_cast<uint8_t>(drivers::ReturnType::LAST)));//fixed, not used now
//      (packet_.return_mode == FIRST_RETURN)
//        ? static_cast<uint8_t>(drivers::ReturnType::FIRST)//drivers::ReturnMode::SINGLE_FIRST
//        : static_cast<uint8_t>(drivers::ReturnType::LAST)));//drivers::ReturnMode::SINGLE_LAST
  }
  return block_pc;
}

drivers::NebulaPointCloudPtr PandarQT128Decoder::convert_dual(size_t block_id)
{
  return convert(block_id);
}

bool PandarQT128Decoder::parsePacket(const pandar_msgs::msg::PandarPacket & pandar_packet)
{
  if (pandar_packet.size != PACKET_SIZE && pandar_packet.size != PACKET_WITHOUT_UDPSEQ_CRC_SIZE) {
    return false;
  }
  const uint8_t * buf = &pandar_packet.data[0];

//  size_t index = 0;
  int index = 0;
  // Parse 12 Bytes Header
  packet_.header.sob = (buf[index] & 0xff) << 8 | ((buf[index + 1] & 0xff));
  packet_.header.chProtocolMajor = buf[index + 2] & 0xff;
  packet_.header.chProtocolMinor = buf[index + 3] & 0xff;
  packet_.header.chLaserNumber = buf[index + 6] & 0xff;
  packet_.header.chBlockNumber = buf[index + 7] & 0xff;
  packet_.header.chReturnType = buf[index + 8] & 0xff;// First Block Return (Reserved)
  packet_.header.chDisUnit = buf[index + 9] & 0xff;
  index += HEAD_SIZE;

  if (packet_.header.sob != 0xEEFF) {
    // Error Start of Packet!
    return false;
  }

//  std::cout << "packet_.header.chBlockNumber=" << packet_.header.chBlockNumber << std::endl;
//  std::cout << "packet_.header.chLaserNumber=" << packet_.header.chLaserNumber << std::endl;
  for (size_t block = 0; block < static_cast<size_t>(packet_.header.chBlockNumber); block++) {
    packet_.blocks[block].azimuth = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    index += BLOCK_HEADER_AZIMUTH;

    for (size_t unit = 0; unit < packet_.header.chLaserNumber; unit++) {
      unsigned int unRange = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      packet_.blocks[block].units[unit].distance =
        (static_cast<float>(unRange * packet_.header.chDisUnit)) / 1000.f;
      packet_.blocks[block].units[unit].intensity = (buf[index + 2] & 0xff);
      packet_.blocks[block].units[unit].confidence = (buf[index + 3] & 0xff);
      index += UNIT_SIZE;
    }
  }


  index += SKIP_SIZE;
  packet_.mode_flag = buf[index] & 0xff;//Mode Flag
  index += MODE_FLAG_SIZE;
  index += RESERVED3_SIZE;
  packet_.return_mode = buf[index] & 0xff;//Return Mode
  index += RETURN_MODE_SIZE;

  packet_.t.tm_year = (buf[index + 0] & 0xff) + 100;
  packet_.t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet_.t.tm_mday = buf[index + 2] & 0xff;
  packet_.t.tm_hour = buf[index + 3] & 0xff;
  packet_.t.tm_min = buf[index + 4] & 0xff;
  packet_.t.tm_sec = buf[index + 5] & 0xff;
  packet_.t.tm_isdst = 0;
  index += UTC_SIZE;

  packet_.usec = (buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8 |
                 ((buf[index + 2] & 0xff) << 16) | ((buf[index + 3] & 0xff) << 24);
  index += TIMESTAMP_SIZE;

  // in case of time error
  if (packet_.t.tm_year >= 200) {
    packet_.t.tm_year -= 100;
  }

  return true;
}
}  // namespace pandar_qt_128
}  // namespace drivers
}  // namespace nebula