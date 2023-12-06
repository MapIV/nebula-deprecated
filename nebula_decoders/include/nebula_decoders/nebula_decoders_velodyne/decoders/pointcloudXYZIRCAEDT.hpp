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

#ifndef __POINTCLOUDXYZIRCAEDT_H
#define __POINTCLOUDXYZIRCAEDT_H

#include <pcl/point_cloud.h>
#include <nebula_decoders/nebula_decoders_velodyne/decoders/datacontainerbase.hpp>

#include "nebula_common/point_types.hpp"
//#include <nebula_decoders/nebula_decoders_velodyne/decoders/point_types.hpp>

namespace nebula
{
namespace drivers
{
class PointcloudXYZIRCAEDT : public drivers::DataContainerBase
{
public:
  pcl::PointCloud<drivers::PointXYZIRCAEDT>::Ptr pc;

  PointcloudXYZIRCAEDT() : pc(new pcl::PointCloud<drivers::PointXYZIRCAEDT>) {}

  virtual void addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & return_type, const uint16_t & ring, const uint16_t & azimuth,
    const float & distance, const float & intensity,
    const double & time_stamp) override;
  virtual void addPoint(
    const float & x, const float & y, const float & z,
    const uint8_t & intensity,const uint8_t & return_type, const uint16_t & channel,
    const float & azimuth, const float & elevation, const float & distance,
    const uint32_t & time_stamp) override;
};
}  // namespace drivers
}  // namespace nebula
#endif
