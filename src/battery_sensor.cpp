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

#include "battery_sensor.h"

namespace woz_simulated_sensors
{

BatterySensor::BatterySensor(const std::string& id,
                             const std::string& description, double min,
                             double max, double mean, double sigma,
                             bool use_map) :
        Sensor(id, description, min, max, mean, sigma, use_map),
        initial_level_(mean),
        time_till_discharge_(45 * 60), // 45 minutes
        sub_clock_(nh_.subscribe("/RunClock", 5, &BatterySensor::clockCb, this))
{
}

void BatterySensor::clockCb(const std_msgs::DurationConstPtr& msg)
{
  current_duration_ = msg->data;

  ros::Duration elapsed_sinse_charge = current_duration_ - time_charged_;
  double a = elapsed_sinse_charge.toSec() / time_till_discharge_.toSec();
  updateDistribution(initial_level_ - (initial_level_ - min_) * a);
}

SensorStatus BatterySensor::getValueAt(double x, double y)
{
  SensorStatus status = Sensor::getValueAt(x, y);
  // Update the message so that warn level is only when the value close to min
  if (status.value < status.min * 1.1)
  {
    status.level = status.WARN;
  }
  else
  {
    status.level = status.OK;
  }
  return status;
}

void BatterySensor::rechargeBattery()
{
  sensor_msg_.nominal = initial_level_;
}
} /* namespace woz_simulated_sensors */
