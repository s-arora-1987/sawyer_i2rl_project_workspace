
#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <memory>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
      // Forward declare private data class.
  class BlinkVisualPluginPrivate;

  /// \brief An example plugin for a contact sensor.
  class ContactPlugin : public SensorPlugin, public VisualPlugin
  {
    /// \brief Constructor.
    public: ContactPlugin();

    /// \brief Destructor.
    public: virtual ~ContactPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    /// \brief Pointer to the contact sensor
    private: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    private: event::ConnectionPtr updateConnection;

    /// \brief Callback to receive world statistics.
    private: void OnWorldStats(ConstWorldStatisticsPtr &_msg);

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<BlinkVisualPluginPrivate> dataPtr;

  };
}
#endif
