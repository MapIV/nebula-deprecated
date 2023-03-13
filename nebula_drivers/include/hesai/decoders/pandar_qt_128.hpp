#pragma once
/**
 * Pandar QT128
 */
#include <cstddef>
#include <cstdint>
#include <ctime>

namespace nebula
{
namespace drivers
{
namespace pandar_qt_128
{
constexpr uint16_t MAX_AZIMUTH_STEPS = 900;  // High Res mode
//constexpr float DISTANCE_UNIT = 0.004f;       // 4mm
constexpr float MIN_RANGE = 0.05;
constexpr float MAX_RANGE = 50.0;

// Head
constexpr size_t HEAD_SIZE = 12;
constexpr size_t PRE_HEADER_SIZE = 6;
constexpr size_t HEADER_SIZE = 6;
// Body
constexpr size_t BLOCKS_PER_PACKET = 2;
constexpr size_t BLOCK_HEADER_AZIMUTH = 2;
constexpr size_t LASER_COUNT = 128;
constexpr size_t UNIT_SIZE = 4;
constexpr size_t CRC_SIZE = 4;
constexpr size_t BLOCK_SIZE = UNIT_SIZE * LASER_COUNT + BLOCK_HEADER_AZIMUTH;
constexpr size_t BODY_SIZE = BLOCK_SIZE * BLOCKS_PER_PACKET + CRC_SIZE;
// Functional Safety
constexpr size_t FS_VERSION_SIZE = 1;
constexpr size_t LIDAR_STATE_SIZE = 1;
constexpr size_t FAULT_SIZE = 1;
constexpr size_t OUT_FAULT_SIZE = 2;
constexpr size_t RESERVED_SIZE = 8;
constexpr size_t CRC2_SIZE = 4;
constexpr size_t PACKET_FS_SIZE = 17;
// Tail
constexpr size_t RESERVED2_SIZE = 5;
constexpr size_t MODE_FLAG_SIZE = 1;
constexpr size_t RESERVED3_SIZE = 6;
constexpr size_t RETURN_MODE_SIZE = 1;
constexpr size_t MOTOR_SPEED_SIZE = 2;
constexpr size_t UTC_SIZE = 6;
constexpr size_t TIMESTAMP_SIZE = 4;
constexpr size_t FACTORY_SIZE = 1;
constexpr size_t SEQUENCE_SIZE = 4;
constexpr size_t CRC3_SIZE = 4;
constexpr size_t PACKET_TAIL_SIZE = 34;
constexpr size_t PACKET_TAIL_WITHOUT_UDPSEQ_CRC_SIZE = 26;

// Cyber Security
constexpr size_t SIGNATURE_SIZE = 32;

constexpr size_t SKIP_SIZE = CRC_SIZE + PACKET_FS_SIZE + RESERVED2_SIZE;

// All
constexpr size_t PACKET_SIZE =
  HEAD_SIZE + BODY_SIZE + PACKET_FS_SIZE + PACKET_TAIL_SIZE + SIGNATURE_SIZE;
constexpr size_t PACKET_WITHOUT_UDPSEQ_CRC_SIZE =
  HEAD_SIZE + BODY_SIZE + PACKET_FS_SIZE + PACKET_TAIL_WITHOUT_UDPSEQ_CRC_SIZE;

constexpr uint32_t SINGLE_FIRST_RETURN = 0x33;
constexpr uint32_t SINGLE_SECOND_RETURN = 0x34;
constexpr uint32_t SINGLE_STRONGEST_RETURN = 0x37;
constexpr uint32_t SINGLE_LAST_RETURN = 0x38;
constexpr uint32_t DUAL_LAST_STRONGEST_RETURN = 0x39;
constexpr uint32_t DUAL_FIRST_LAST_RETURN = 0x3B;
constexpr uint32_t DUAL_FIRST_STRONGEST_RETURN = 0x3C;
constexpr uint32_t DUAL_STRONGEST_2ndSTRONGEST_RETURN = 0x3E;
constexpr uint32_t DUAL_FIRST_SECOND_RETURN = 0x3A;

struct Header
{
  uint16_t sob;             // 0xFFEE 2bytes
  uint8_t chProtocolMajor;  // Protocol Version Major 1byte
  uint8_t chProtocolMinor;  // Protocol Version Minor 1byte
  uint8_t chLaserNumber;    // laser number 1byte
  uint8_t chBlockNumber;    // block number 1byte
  uint8_t chDisUnit;        // Distance unit, 4mm
  uint8_t chReturnType;     // return mode 1 byte  when dual return 0-Single Return
                            // 1-The first block is the 1 st return.
                            // 2-The first block is the 2 nd return
  uint8_t chFlags;          // [6] channel customization: 1-Selected channels, 0-All channels
                            // [3] digital signature: 1-YES, 0-NO
                            // [2] functional safety: 1-YES, 0-NO
                            // [1] IMU: 1-YES, 0-NO
                            // [0] UDP sequence: 1-YES, 0-NO
};

struct Unit
{
  double distance;
  uint16_t intensity;
  uint16_t confidence;
};

struct Block
{
  uint16_t azimuth;  // packet angle,Azimuth = RealAzimuth * 100
  Unit units[LASER_COUNT];
};

struct Packet
{
  Header header;
  Block blocks[BLOCKS_PER_PACKET];
  uint32_t usec;  // ms
  uint32_t mode_flag;
  uint32_t return_mode;
  tm t;
};
}  // namespace pandar_qt_128
}  // namespace drivers
}  // namespace nebula
