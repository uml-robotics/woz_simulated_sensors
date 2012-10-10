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

#ifndef SENSOR_H_
#define SENSOR_H_

#include "sensor_map.h"

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <woz_simulated_sensors/SensorStatus.h>


namespace woz_simulated_sensors
{

/*
 *
 */
class Sensor
{
public:
  /**
   *
   * @param id Identification string, used in message naming and map lookup.
   * @param description Human readable description.
   * @param min Minimum value
   * @param max Maximum value
   * @param mean Normal reading value
   * @param noise_sigma Gussian noise sigma, 0 - no noise
   * @param use_map True to load the initial map, if map is used, mean discarded.
   */
  Sensor(const std::string& id, const std::string & description, double min,
         double max, double mean, double sigma, bool use_map = false);

  void updateDistribution(double mean);
  void updateDistribution(double mean, double sigma);
  /**
   * Produce the simulated sensor value at (x, y);
   * @param x
   * @param y
   * @return
   */
  SensorStatus getValueAt(double x, double y);
  /**
   * Used for simulating sensor malfunction.
   * @param delta The delta gets added to the normal value.
   */
  void setMalfunctionDelta(double delta);
protected:
  SensorStatus sensor_msg_;
  std::string id_;
  std::string description_;
  double min_;
  double max_;
  boost::shared_ptr<SensorMap> sensor_map_;

  boost::mt19937 rng_;
  boost::normal_distribution<> nd_;
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor_;

  static int seed;

  double malfunction_delta_;

};

} /* namespace woz_simulated_sensors */
#endif /* SENSOR_H_ */
