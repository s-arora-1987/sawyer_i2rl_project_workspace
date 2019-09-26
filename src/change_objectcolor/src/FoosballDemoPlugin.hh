/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GAZEBO_BLINKVISUALPLUGIN_HH_
#define GAZEBO_BLINKVISUALPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>

#include <mutex>

#include <gazebo/common/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>

#include <ros/ros.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <std_msgs/String.h>


namespace gazebo
{
  // Forward declare private data class.
  class BlinkVisualPluginPrivate;

  /// \brief Plugin that makes a visual blink between two colors. See the
  /// example usage below:
  ///
  /// \verbatim
  ///    <plugin name="blink" filename="libBlinkVisualPlugin.so">
  ///
  ///      <!-- First RGBA color, each number from 0 to 1. Defaults to red. -->
  ///      <color_a>1 0 0 1</color_a>
  ///
  ///      <!-- Second RGBA color. Defaults to black. -->
  ///      <color_a>0 0 0 1</color_a>
  ///
  ///      <!-- Period in seconds. Defaults to 1 s. -->
  ///      <period>1</period>
  ///
  ///      <!-- True to use wall time, false to use sim time.
  ///           Defaults to false. -->
  ///      <use_wall_time>true</use_wall_time>
  ///
  ///    </plugin>
  /// \endverbatim
  ///
  /// See worlds/blink_visual.world for a complete example.
  class GAZEBO_VISIBLE BlinkVisualPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: BlinkVisualPlugin();

    /// \brief Destructor.
    public: ~BlinkVisualPlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _visual,
        sdf::ElementPtr _sdf);

    /// \brief Update the plugin once every iteration of simulation.
//    private: void Update();
    private: void Update(const std_msgs::String::ConstPtr &cmd_msg);

    /// \brief Callback to receive world statistics.
//    private: void OnWorldStats(ConstWorldStatisticsPtr &_msg);

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<BlinkVisualPluginPrivate> dataPtr;
    private: ros::NodeHandle *rosnode_;

    private: GazeboRosPtr gazebo_ros_;
    private: std::string robot_namespace_;
    private: std::string command_topic_;
    private: ros::CallbackQueue queue_;
    private: ros::Subscriber cmd_string_subscriber_;
    private: boost::thread callbackQueueThread_;
    private: void QueueThread();

  };
}
#endif
