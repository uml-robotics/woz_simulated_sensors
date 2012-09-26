#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

namespace simulated_sensors
{
class Sensors
{
public:
  Sensors();
private:

  ros::NodeHandle nh_;

};

} // namespace simulated_sensors

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensors");
  simulated_sensors::Sensors s;
  ros::spin();
  return 0;
}

namespace simulated_sensors
{
Sensors::Sensors()
{
  ros::ServiceClient client = nh_.serviceClient<nav_msgs::GetMap>(
      "map_co2/static_map");

  nav_msgs::GetMap srv;
  client.call(srv);
  ROS_INFO_STREAM("got map length "<< srv.response.map.data.size());

}

} // namespace simulated_sensors
