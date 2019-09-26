
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"

#include <iostream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/String.h>


namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();
//    private: virtual void OnUpdate2();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    private: std::string robot_namespace_;
    // Create our node for communication
    private: ros::NodeHandle* rosnode_;
    private: std_msgs::String str;

    private: ros::Publisher contact_pub_;

  };
}
#endif
