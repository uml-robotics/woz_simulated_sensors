#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_map.h"

namespace simulated_sensors
{
class Sensors
{
public:
  Sensors();

  void update();

private:
  SensorMap sensor_map_;

  // ROS
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  geometry_msgs::PoseStamped zero_pose_;
  std::string base_frame_id_;
  std::string map_frame_id_;
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
        sensor_map_("map_co2/static_map")
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

  double &map_x = map_pose.pose.position.x;
  double &map_y = map_pose.pose.position.y;

  ROS_INFO_STREAM(" = "<<sensor_map_.getValueAt(map_x, map_y));

}

} // namespace simulated_sensors
