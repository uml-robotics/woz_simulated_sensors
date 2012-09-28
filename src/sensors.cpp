#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor.h"
#include <woz_msgs/Sensors.h>
#include <woz_simulated_sensors/SensorArray.h>

namespace woz_simulated_sensors
{
class Sensors
{
public:
  Sensors();

  void update();

private:
  Sensor sensor_co2;

  // ROS
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  geometry_msgs::PoseStamped zero_pose_;
  std::string base_frame_id_;
  std::string map_frame_id_;
  ros::Publisher sensors_pub_;
};

} // namespace woz_simulated_sensors

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensors");
  woz_simulated_sensors::Sensors s;
  ros::Rate r(2);
  while (ros::ok())
  {
    s.update();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}

namespace woz_simulated_sensors
{
Sensors::Sensors()
: sensor_co2("co2", "Carbon Dioxide", 0, 100, 50, 2, true )
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

  sensors_pub_ = nh_.advertise<woz_simulated_sensors::SensorArray>("sensors", 1);
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

  SensorArray msg;
  msg.header.stamp = ros::Time::now();
  msg.sensors.push_back(sensor_co2.getValueAt(x,y));

  sensors_pub_.publish(msg);
}

} // namespace woz_simulated_sensors
