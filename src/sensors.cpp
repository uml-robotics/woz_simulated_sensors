#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor.h"
#include "location_sensor.h"
#include "battery_sensor.h"
#include <woz_simulated_sensors/SensorArray.h>

namespace woz_simulated_sensors
{
struct RadiationFSM;
class Sensors
{
public:
  Sensors();

  void update();

private:
  // NeutronRAE-II
//  Sensor sensor_rad_neutron_gamma;
  Sensor sensor_temperature;

  // MultiRAE Pro
  Sensor sensor_rad_gamma;
  Sensor sensor_co2;
  Sensor sensor_electrochem;
  Sensor sensor_combust_gases;
  Sensor sensor_volatile_organic;
  RadiationFSM * radiation_fsm_;

  // Heartbeat
  Sensor sensor_heartbeat;

  // Location Sensor
  LocationSensor sensor_locations;

  // Fake Battery Sensor
  BatterySensor sensor_battery;

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
struct RadiationFSM
{
  typedef enum
  {
    Initial, FirstAtEngineering, FirstOutOfEngineering, SecondAtEngineering,
    SecondOutOfEngineering
  } States;
  States state;
  RadiationFSM() :
          state(Initial)
  {
  }

  bool isMalfunctioningAfterUpdate(const std::string &location)
  {
    switch (state)
    {
      case Initial:
      {
        if (location == "Engineering Room")
          state = FirstAtEngineering;
        break;
      }
      case FirstAtEngineering:
      {
        if (location == "South Wing by the Engineering Room")
          state = FirstOutOfEngineering;
        break;
      }
      case FirstOutOfEngineering:
      {
        if (location == "Engineering Room")
          state = SecondAtEngineering;
        break;
      }
      case SecondAtEngineering:
      {
        if (location == "South Wing by the Engineering Room")
          state = SecondOutOfEngineering;
        break;
      }
      case SecondOutOfEngineering:
      {
        break;
      }
    }
    return state == SecondOutOfEngineering;
  }
};

Sensors::Sensors() :
        // NeutronRAE-II
//        sensor_rad_neutron_gamma("neutron_gamma", "Gamma / Neutron, cpm", 0,
//                                 100, 40 / 60.0, 0.02),
        sensor_temperature("temperature", "Ambient Temp., F", 0, 100, 71, 0.5,
                           true), // F

        // MultiRAE Pro
        sensor_rad_gamma("gamma", "Gamma, mSv", 1e-3, 9900, 4, 0.05),
        sensor_co2("co2", "Carbon Dioxide, ppm", 0, 50000, 500, 10),
        sensor_electrochem("electrochem", "Ammonia, ppm", 0, 100, 0, 0.0, true),
        sensor_combust_gases("combust_gases", "Methane, ppm", 0, 100, 1, 0.01),
        sensor_volatile_organic("volatile_organic", "Cleaning Chem, ppm", 0,
                                100, 1.3, 0.01),

        // Heartbeat
        sensor_heartbeat("heartbeat", "Heartbeat, bpm", 0, 100, 70, 1, true),
        sensor_locations("locations", "Location", 0, 100, 70, 0, true, { {
            0, "Uninitialized"}, //
            {11, "Engineering Room"}, //
            {13, "Engineering Room by the Hallway"}, //
            {28, "South Wing by the Engineering Room"}, //
            {29, "South Wing between the Engineering and the Research Lab"}, //
            {31, "South Wing"}, //
            {22, "South Wing by the Research Lab"}, //
            {32, "South Wing by the South Entrance"}, //
            {35, "South Wing by the Cafeteria"}, //
            {19, "Research Lab"}, //
            {21, "Research Lab by the Hallway"}, //
            {38, "Cafeteria by the South Wing"}, //
            {36, "Cafeteria by the Elevators"}, //
            {34, "Cafeteria by the Control Room"}, //
            {33, "Cafeteria"}, //
            {46, "Control Room"}, //
            {47, "Control Room by the Hallway"}, //
//            {41, "North Wing by the Cafeteria"}, //
//            {44, "North Wing between the Control Room and the Conference Room"}, //
//            {40, "North Wing"}, //
//            {42, "North Wing by the North Entrance"}, //
//            {43, "North Wing by the Conference Room"}, //
//            {56, "Conference Room"}, //
//            {57, "Conference Room by Hallway"}, //
                         }),
        sensor_battery("battery", "Battery, V", 20, 31, 29, 0.3)

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

  sensors_pub_ = nh_.advertise<woz_simulated_sensors::SensorArray>("sensors",
                                                                   1);
  radiation_fsm_ = new RadiationFSM;
  sensor_heartbeat.setPrecision(0);
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
  msg.sensors =
  {
//    sensor_rad_neutron_gamma.getValueAt(x, y),
    sensor_temperature.getValueAt(x, y),
    sensor_rad_gamma.getValueAt(x, y),
    sensor_co2.getValueAt(x, y),
    sensor_electrochem.getValueAt(x, y),
    sensor_combust_gases.getValueAt(x, y),
    sensor_volatile_organic.getValueAt(x, y),
    sensor_heartbeat.getValueAt(x, y),
    sensor_locations.getValueAt(x, y)
    , sensor_battery.getValueAt(x, y)
  };

  sensors_pub_.publish(msg);

  // Simulate radiation sensor failure.
  if (radiation_fsm_->isMalfunctioningAfterUpdate(
      sensor_locations.getValueAt(x, y).message))
  {
    sensor_rad_gamma.setMalfunctionDelta(478);
  }
  else
  {
    sensor_rad_gamma.setMalfunctionDelta(0);
  }
}

} // namespace woz_simulated_sensors
