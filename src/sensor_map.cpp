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

#include "sensor_map.h"

#include <nav_msgs/GetMap.h>

namespace simulated_sensors{


SensorMap::SensorMap(std::string service_name)
{
  ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>(service_name);
  while (!client.exists() && ros::ok())
  {
    ROS_INFO_STREAM("Waiting for '"<< service_name << "' service.");
    ros::Duration(2).sleep();
  }
  nav_msgs::GetMap srv;
  client.call(srv);
  map_ = srv.response.map;
  ROS_INFO_STREAM(
      "Received map, "<<map_.info.width <<" x "<<map_.info.height<<" px.");
}

int SensorMap::getValueAt(double x, double y)
{
  x += map_.info.origin.position.x;
  y += map_.info.origin.position.y;

  int px = map_.info.width + x / map_.info.resolution;
  int py = map_.info.height + y / map_.info.resolution;

  // Check for boundaries
  if (px < 0 || py < 0 || px > (int)map_.info.width
      || py > (int)map_.info.height)
  {
    return -1;
  }
  else
  {
    return (unsigned char)map_.data[px + py * map_.info.width];
  }
}


} /* namespace simulated_sensors */
