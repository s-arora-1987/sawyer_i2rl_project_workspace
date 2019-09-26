
#include "ContactPlugin_Publisher.hh"

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
//  std::cout << "name for contact plugin." << _sdf->GetAttribute("name")->GetAsString() << std::endl;

  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
//  std::cout << "publisher this->robot_namespace_" << this->robot_namespace_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("bumper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->contact_pub_ = this->rosnode_->advertise<std_msgs::String>("contact_msg", 1);

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
//  std::cout << "Connected to the sensor update event";
}

/////////////////////////////////////////////////
// publish a contact msg when grasped
// will it change every soft object's color yes
// not if name of object is in msg
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
//  std::cout << "in OnUpdate()";
  contacts = this->parentSensor->Contacts();
  std::string s2 = "scara";
  std::string s3 = "sawyer";
  std::string s4 = "conveyor";
  std::string s1;
  bool b = true;

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    s1 = contacts.contact(i).collision2();

    if ( s1.find(s4) != std::string::npos ) { // if object in contact with conveyor

        b = false;
    }
  }

  if ( b == true ) {

      for (unsigned int i = 0; i < contacts.contact_size(); ++i)
      {
        s1 = contacts.contact(i).collision2();

    //    std::cout << "Collision between[" << contacts.contact(i).collision1()
    //              << "] and [" << contacts.contact(i).collision2() << "]\n";
        if ( s1.find(s2) != std::string::npos || s1.find(s3) != std::string::npos ){

          std_msgs::String str;
          str.data = this->robot_namespace_;
    //      std::cout << "publishing on contact " << str.data;
          this->contact_pub_.publish(str);

        } // else topic should be empty

      }

  }
}
