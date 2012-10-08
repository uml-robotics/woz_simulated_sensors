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

#include "sensor.h"

namespace woz_simulated_sensors
{

int Sensor::seed = clock();

Sensor::Sensor(const std::string& id, const std::string& description,
               double min, double max, double mean, double noise_sigma,
               bool use_map) :
        min_(min),
        max_(max),
        rng_(static_cast<unsigned int>(std::clock()) + (seed *= 13 * seed)),
        nd_(mean, noise_sigma),
        var_nor_(rng_, nd_)
{
  sensor_msg_.hardware_id = id;
  sensor_msg_.name = description;
  sensor_msg_.min = min;
  sensor_msg_.max = max;
  sensor_msg_.nominal = mean;

//  updateDistribution(mean, noise_sigma);

  if (use_map)
  {
    std::string map_service("/map_" + id + "/static_map");
    sensor_map_.reset(new SensorMap(map_service));
    ROS_INFO_STREAM("Sensor: "<< description <<" using "<< map_service);
  }
  else
  {
    ROS_INFO_STREAM(
        "Sensor: "<< description <<" using value "<< mean<<" (sigma "<< noise_sigma<<")");
  }
}

void Sensor::updateDistribution(double mean)
{
  updateDistribution(mean, nd_.sigma());
}

void Sensor::updateDistribution(double mean, double sigma)
{
  nd_ = boost::normal_distribution<>(mean, sigma);
  var_nor_.distribution() = nd_;
}

SensorStatus Sensor::getValueAt(double x, double y)
{
  // Get initial value
  if (sensor_map_)
  {
    sensor_msg_.value = sensor_map_->getValueAt(x, y);
    // Adjust according to max/min
    sensor_msg_.value = sensor_msg_.value / 255.0 * (max_ - min_) + min_;
    // apply noise
    sensor_msg_.value += var_nor_() - nd_.mean();
  }
  else
  {
    sensor_msg_.value = var_nor_();

  }

  // Set warning if value is 10% higher than normal.
  if (sensor_msg_.value > nd_.mean() * 1.1)
  {
    sensor_msg_.level = SensorStatus::WARN;
  }
  else
  {
    sensor_msg_.level = SensorStatus::OK;
  }

  // check limits
  if (sensor_msg_.value < min_)
  {
    sensor_msg_.value = min_;
  }
  else if (sensor_msg_.value > max_)
  {
    sensor_msg_.value = max_;
  }

  return sensor_msg_;
}

} /* namespace woz_simulated_sensors */
