#include "ContactPlugin_ChangeVisual.hh"
#include <mutex>
#include <gazebo/common/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

namespace gazebo
{
  /// \internal
  /// \class BlinkVisualPlugin BlinkVisualPlugin.hh
  /// \brief Private data for the BlinkVisualPlugin class.
  class BlinkVisualPluginPrivate
  {
    /// \brief Visual whose color will be changed.
    public: rendering::VisualPtr visual;

    /// \brief Connects to rendering update event.
//    public: event::ConnectionPtr updateConnection;

    /// \brief First color.
    public: common::Color colorA;

    /// \brief Second color.
    public: common::Color colorB;

    /// \brief Time taken by a full cycle.
    public: common::Time period;

    /// \brief Time the current cycle started.
//    public: common::Time cycleStartTime;

    /// \brief The current simulation time, got through the world_stats topic.
//    public: common::Time currentSimTime;

    /// \brief Node used for communication.
    public: transport::NodePtr node;

    /// \brief Node used for communication.
    public: std::mutex mutex;

    /// \brief True to use wall time, false to use sim time.
    public: bool useWallTime;

    /// \brief Subscriber to the world statistics topic.
//    public: transport::SubscriberPtr statsSub;
  };
}

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin(), dataPtr(new BlinkVisualPluginPrivate)
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
    if (this->dataPtr->node)
    this->dataPtr->node->Fini();

}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{

   if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  if (!_sdf->HasElement("sdf"))
  {
    gzerr << "An <sdf> tag can't be found within the plugin." << std::endl;
    return;
  }
  auto sdfElem = _sdf->GetElement("sdf");

  // Get color A
  this->dataPtr->colorA.Set(1, 0, 0, 1);
//  if (sdfElem->HasElement("color_a"))
//    this->dataPtr->colorA = sdfElem->Get<common::Color>("color_a");

  // Get color B/src/change_objectcolor/src/ContactPlugin_ChangeVisual.cc:176:18: error: ‘class gazebo::BlinkVisualPluginPrivate’ has no member named ‘currentS
  this->dataPtr->colorB.Set(0, 0, 0, 1);
//  if (sdfElem->HasElement("color_b"))
//    this->dataPtr->colorB = sdfElem->Get<common::Color>("color_b");

  // Get the period
  this->dataPtr->period.Set(0.1);
  if (sdfElem->HasElement("period"))
    this->dataPtr->period = sdfElem->Get<double>("period");

//  if (this->dataPtr->period <= 0)
//  {
//    gzerr << "Period can't be lower than zero." << std::endl;
//    return;
//  }

  // Get whether to use wall time or sim time
  this->dataPtr->useWallTime = false;
//  if (sdfElem->HasElement("use_wall_time"))
//    this->dataPtr->useWallTime = sdfElem->Get<bool>("use_wall_time");

  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    std::string s1 = contacts.contact(i).collision2();
    std::string s2 = "scara";
    if ( s1.find(s2) != std::string::npos ){
      std::cout << "Collision between[" << contacts.contact(i).collision1()
              << "] and [" << contacts.contact(i).collision2() << "]\n";
    }

  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!this->dataPtr->visual)
  {
    gzerr << "The visual is null." << std::endl;
    return;
  }
  common::Color from;
  common::Color to;
  from = this->dataPtr->colorA;
  to = this->dataPtr->colorB;
  double red = to.r;
  double green = to.g;
  double blue = to.b;
  double alpha = to.a;

  common::Color color(red, green, blue, alpha);

//  this->dataPtr->visual->SetDiffuse(color);
//  this->dataPtr->visual->SetAmbient(color);
//  this->dataPtr->visual->SetTransparency(1-color.a);


}
/////////////////////////////////////////////////
void ContactPlugin::OnWorldStats(ConstWorldStatisticsPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
//  this->dataPtr->currentSimTime = msgs::Convert(_msg->sim_time());
}
