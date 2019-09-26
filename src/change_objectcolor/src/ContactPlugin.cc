
#include "ContactPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{

  std::cout << "name for contact plugin." << _sdf->GetAttribute("name")->GetAsString() << std::endl;
//  sdf::ElementPtr _sdf2 = _sdf->GetParent();
//  sdf::ElementPtr _sdf3 = _sdf2->GetParent();
//  std::cout << "initializes _sdf3." << std::endl;
//  std::cout << "name for sensor parent." << _sdf3->GetDescription() << std::endl;
//  if (!_sdf->HasElement("plugin")) {
//    std::cout << "can't retrieve name for <plugin>." << std::endl;
//    return;
//  }
//  else
//    std::cout << "name for <plugin>." << _sdf->GetElement("plugin")->GetName().c_str() << std::endl;

//  auto sdfElem = _sdf->GetParent()->GetParent()->GetParent();
//  if (!sdfElem->HasElement("link")) {
//    std::cout << "<link> tag can't be found within the 3rd parent of plugin." << std::endl;
//  }
//  else {
//
//    if (!sdfElem->HasElement("model"))
//      std::cout << "no <model>." << std::endl;
//    else
//      std::cout << "name:" << sdfElem->GetElement("model")->GetName().c_str() << std::endl;
//  }

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
}
