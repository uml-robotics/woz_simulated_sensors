/*
 * Copyright (c) 2012, University of Massachusetts Lowell.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of University of Massachusetts Lowell. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Mikhail Medvedev */

#include "location_sensor.h"

namespace woz_simulated_sensors
{

void LocationSensor::setMapping(
    const std::map<int, std::string>& location_mapping)
{
  location_mapping_ = location_mapping;
}

LocationSensor::LocationSensor(
    const std::string& id, const std::string& description, double min,
    double max, double mean, double sigma, bool use_map,
    const std::map<int, std::string>location_mapping) :
        Sensor(id, description, min, max, mean, sigma, use_map)
{
  location_mapping_ = location_mapping;
}

SensorStatus LocationSensor::getValueAt(double x, double y)
{
  SensorStatus status = Sensor::getValueAt(x, y);
  // Truncate value to int
  status.value =(int)status.value;
  auto location = location_mapping_.find((int)status.value);
  if (location != location_mapping_.end())
  {
    status.message = location_mapping_[(int)status.value];
  }
  else
  {
    status.message = "Unknown Location";
  }
  return status;
}

} /* namespace woz_simulated_sensors */
