//#include <QDebug>

#include "my_plugin.h"
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(ModelPush)

void ModelPush::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
    //qDebug() << "loading plugin...";
    // Store the pointer to the model
    this->model = _parent;
    // Listen to the update event. This event is broadcast pre-render update???
    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&ModelPush::OnUpdate, this));
}

// Called by the world update start event
void ModelPush::OnUpdate()
{
    static float r=0.0, increment=0.01;
    //qDebug() << r;
    if (r > 1.0) {
        increment = -0.01;
        r = 1.0;
    }
    if (r < 0.0) {
        r = 0.0;
        increment = 0.01;
    }
    //
    common::Color c(r, 0.0, 0.0);
    this->model->SetAmbient(c);
    this->model->SetDiffuse(c);
    //
    r += increment;
}