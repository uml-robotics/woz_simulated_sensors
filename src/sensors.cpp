#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_map.h"
#include <woz_msgs/Sensors.h>

namespace simulated_sensors
{
class Sensors
{
public:
  Sensors();

  void update();

private:
  SensorMap sensor_map_co2;
  SensorMap sensor_map_o2;
  SensorMap sensor_map_temperature;
  SensorMap sensor_map_ion_rad;
  SensorMap sensor_map_person;

  // ROS
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  geometry_msgs::PoseStamped zero_pose_;
  std::string base_frame_id_;
  std::string map_frame_id_;
  ros::Publisher sensors_pub_;
};

} // namespace simulated_sensors

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensors");
  simulated_sensors::Sensors s;
  ros::Rate r(2);
  while (ros::ok())
  {
    s.update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

namespace simulated_sensors
{
Sensors::Sensors() :
        sensor_map_co2("map_co2/static_map"),
        sensor_map_o2("map_o2/static_map"),
        sensor_map_temperature("map_temperature/static_map"),
        sensor_map_ion_rad("map_ion_rad/static_map"),
        sensor_map_person("map_person/static_map")
{

  nh_.param("base_frame_id", base_frame_id_, (std::string)"/base_link");
  nh_.param("map_frame_id", map_frame_id_, (std::string)"/map");

  zero_pose_.header.frame_id = base_frame_id_;
  zero_pose_.pose.orientation.w = 1;
  while (!tf_listener_.waitForTransform(map_frame_id_, base_frame_id_,
                                        ros::Time::now(), ros::Duration(1.0)))
  {
    ROS_INFO_STREAM(
        "Waiting for transform "<<map_frame_id_<<"->"<<base_frame_id_<<".");
  }
  ROS_INFO("Transform ok.");

  sensors_pub_ = nh_.advertise<woz_msgs::Sensors>("sensors", 1);
}

void Sensors::update()
{
  zero_pose_.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped map_pose;
  try
  {
    tf_listener_.waitForTransform(map_frame_id_, base_frame_id_,
                                  ros::Time::now(), ros::Duration(0.5));
    tf_listener_.transformPose(map_frame_id_, zero_pose_, map_pose);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  double &x = map_pose.pose.position.x;
  double &y = map_pose.pose.position.y;

  woz_msgs::Sensors msg;
  msg.co2 = sensor_map_co2.getValueAt(x, y);
  msg.o2 = sensor_map_o2.getValueAt(x, y);
  msg.temperature = sensor_map_temperature.getValueAt(x, y);
  msg.ion_rad = sensor_map_ion_rad.getValueAt(x, y);
  msg.person = sensor_map_person.getValueAt(x, y);

  sensors_pub_.publish(msg);
}

} // namespace simulated_sensors
