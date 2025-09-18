/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <memory>
#include <vector>

#include "perception_msgs/msg/object_list.hpp"
#include "perception_msgs_utils/object_access.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_rendering/objects/shape.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <QColor>

#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <OgreMeshManager.h>
#include <OgreResourceGroupManager.h>

namespace perception_msgs {
namespace displays {

/**
 * \class AttentionCloudDisplay
 * \brief XAI thinking cloud visualization that appears above vehicle when pedestrians detected
 * 
 * This plugin creates a 3D thinking bubble/cloud above the ego vehicle when pedestrians
 * are detected in front of the vehicle (positive X direction within specified range).
 * Features:
 * - 3D cloud geometry that's always camera-facing (billboard)
 * - White fluffy cloud appearance with smooth transitions
 * - Placeholder area for pedestrian icon (future enhancement)
 * - Only shows when pedestrians detected in front of vehicle
 * - Clean, user-friendly XAI visualization for automated vehicles
 */
class AttentionCloudDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList> {
  Q_OBJECT

public:
  AttentionCloudDisplay();
  ~AttentionCloudDisplay() override;

  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

protected:
  void processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;

private slots:
  void updateCloudProperties();

private:
  // Core cloud functionality
  void createThinkingCloud();
  void updateCloudVisibility();
  void destroyCloud();
  
  // Detection logic
  bool hasPedestriansInFront(const perception_msgs::msg::ObjectList& objects);
  
  // Cloud mesh loading
  void loadCloudMesh();
  
  // Cloud animation
  void animateCloud(float dt);

  // Properties
  rviz_common::properties::FloatProperty* detection_range_property_;
  rviz_common::properties::FloatProperty* cloud_height_property_;
  rviz_common::properties::FloatProperty* cloud_size_property_;
  rviz_common::properties::ColorProperty* cloud_color_property_;
  rviz_common::properties::FloatProperty* animation_speed_property_;
  rviz_common::properties::FloatProperty* persistence_time_property_;
  rviz_common::properties::FloatProperty* position_x_offset_property_;
  rviz_common::properties::FloatProperty* position_y_offset_property_;
  rviz_common::properties::BoolProperty* show_placeholder_property_;
  
  // Cloud state
  bool has_pedestrians_detected_;
  bool cloud_visible_;
  float current_opacity_;
  float target_opacity_;
  float animation_time_;
  float persistence_timer_; // Time since last detection
  
  // 3D Objects
  Ogre::SceneNode* cloud_node_;
  Ogre::Entity* cloud_entity_;
  Ogre::MaterialPtr cloud_material_;
  
  // Constants
  static constexpr float DEFAULT_DETECTION_RANGE = 20.0f; // 20 meters in front
  static constexpr float DEFAULT_CLOUD_HEIGHT = 4.0f;     // 4 meters above vehicle
  static constexpr float DEFAULT_CLOUD_SIZE = 1.0f;       // 1 meter diameter
  static constexpr float DEFAULT_ANIMATION_SPEED = 3.0f;   // Fade in/out speed
};

}  // namespace displays
}  // namespace perception_msgs