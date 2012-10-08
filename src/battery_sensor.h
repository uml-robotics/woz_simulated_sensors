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

#ifndef BATTERY_SENSOR_H_
#define BATTERY_SENSOR_H_

#include "sensor.h"
#include <ros/ros.h>
#include <std_msgs/Duration.h>

namespace woz_simulated_sensors
{

/*
 * Subscribes to /RunClock  (Duration) and simulated battery decay with
 * time based on the clock.
 */
class BatterySensor : public Sensor
{
public:
  BatterySensor(const std::string& id, const std::string & description,
                double min, double max, double mean, double sigma,
                bool use_map = false);
  void rechargeBattery();

  SensorStatus getValueAt(double x, double y);

private:
  float initial_level_;
  ros::Duration time_charged_;
  ros::Duration time_till_discharge_; //< How long it takes to get to minimal batt level.

  ros::NodeHandle nh_;
  ros::Subscriber sub_clock_;
  ros::Duration current_duration_;

  void clockCb(const std_msgs::DurationConstPtr & msg);
};

} /* namespace woz_simulated_sensors */
#endif /* BATTERY_SENSOR_H_ */
