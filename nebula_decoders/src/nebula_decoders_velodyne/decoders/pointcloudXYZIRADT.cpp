/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <nebula_decoders/nebula_decoders_velodyne/decoders/pointcloudXYZIRADT.hpp>

namespace nebula
{
namespace drivers
{
void PointcloudXYZIRADT::addPoint(
  const float & x, const float & y, const float & z,
  const uint8_t & return_type, const uint16_t & ring, const uint16_t & azimuth,
  const float & distance, const float & intensity, const double & time_stamp)
{
  drivers::PointXYZIRADT point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.return_type = return_type;
  point.ring = ring;
  point.azimuth = azimuth;
  point.distance = distance;
  point.time_stamp = time_stamp;

  pc->points.push_back(point);
  ++pc->width;
}
void PointcloudXYZIRADT::addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & intensity,const uint8_t & return_type, const uint16_t & channel,
    const float & azimuth, const float & elevation, const float & distance,
    const uint32_t & time_stamp)
{
  drivers::PointXYZIRADT point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.return_type = return_type;
  point.ring = channel;
//  point.elevation = elevation;
  point.azimuth = azimuth;
  point.distance = distance;
  point.time_stamp = time_stamp;

  pc->points.push_back(point);
  ++pc->width;
}
}  // namespace drivers
}  // namespace nebula
