#ifndef MY_PLUGIN_H
#define MY_PLUGIN_H

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Visual.hh>
//#include <common/common.hh>

namespace gazebo
{
class ModelPush : public VisualPlugin
{
public:
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
    // Called by the world update start event
    void OnUpdate();

private:
    // Pointer to the model
    rendering::VisualPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
};
}

#endif // MY_PLUGIN_H
