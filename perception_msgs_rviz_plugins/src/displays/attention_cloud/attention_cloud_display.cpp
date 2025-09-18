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

#include "perception_msgs/displays/attention_cloud/attention_cloud_display.hpp"

#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreEntity.h>
#include <OgreSubEntity.h>
#include <OgreMeshManager.h>
#include <OgreResourceGroupManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/mesh_loader.hpp"

#include <cmath>
#include <algorithm>

namespace perception_msgs {
namespace displays {

AttentionCloudDisplay::AttentionCloudDisplay()
: MessageFilterDisplay<perception_msgs::msg::ObjectList>(),
  has_pedestrians_detected_(false),
  cloud_visible_(false),
  current_opacity_(0.0f),
  target_opacity_(0.0f),
  animation_time_(0.0f),
  persistence_timer_(0.0f),
  cloud_node_(nullptr),
  cloud_entity_(nullptr)
{
  // Initialize properties
  detection_range_property_ = new rviz_common::properties::FloatProperty(
    "Detection Range", DEFAULT_DETECTION_RANGE,
    "Range in front of vehicle to detect pedestrians (meters)",
    this, SLOT(updateCloudProperties()));
  detection_range_property_->setMin(5.0f);
  detection_range_property_->setMax(50.0f);

  cloud_height_property_ = new rviz_common::properties::FloatProperty(
    "Cloud Height", DEFAULT_CLOUD_HEIGHT,
    "Height of thinking cloud above vehicle (meters)",
    this, SLOT(updateCloudProperties()));
  cloud_height_property_->setMin(1.0f);
  cloud_height_property_->setMax(10.0f);

  cloud_size_property_ = new rviz_common::properties::FloatProperty(
    "Cloud Size", DEFAULT_CLOUD_SIZE,
    "Diameter of the thinking cloud (meters)",
    this, SLOT(updateCloudProperties()));
  cloud_size_property_->setMin(0.5f);
  cloud_size_property_->setMax(5.0f);

  cloud_color_property_ = new rviz_common::properties::ColorProperty(
    "Cloud Color", QColor(255, 255, 255),
    "Color of the thinking cloud",
    this, SLOT(updateCloudProperties()));

  animation_speed_property_ = new rviz_common::properties::FloatProperty(
    "Animation Speed", DEFAULT_ANIMATION_SPEED,
    "Speed of cloud fade in/out animation",
    this, SLOT(updateCloudProperties()));
  animation_speed_property_->setMin(0.1f);
  animation_speed_property_->setMax(10.0f);

  persistence_time_property_ = new rviz_common::properties::FloatProperty(
    "Persistence Time", 2.0f,
    "Time cloud stays visible after pedestrians disappear (seconds)",
    this, SLOT(updateCloudProperties()));
  persistence_time_property_->setMin(0.5f);
  persistence_time_property_->setMax(10.0f);

  position_x_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position X Offset", 0.0f,
    "Manual X position offset for cloud positioning (meters)",
    this, SLOT(updateCloudProperties()));
  position_x_offset_property_->setMin(-5.0f);
  position_x_offset_property_->setMax(5.0f);

  position_y_offset_property_ = new rviz_common::properties::FloatProperty(
    "Position Y Offset", 0.0f,
    "Manual Y position offset for cloud positioning (meters)",
    this, SLOT(updateCloudProperties()));
  position_y_offset_property_->setMin(-5.0f);
  position_y_offset_property_->setMax(5.0f);

  show_placeholder_property_ = new rviz_common::properties::BoolProperty(
    "Show Icon Placeholder", true,
    "Show placeholder for pedestrian icon inside cloud",
    this, SLOT(updateCloudProperties()));
}

AttentionCloudDisplay::~AttentionCloudDisplay()
{
  destroyCloud();
  
  delete detection_range_property_;
  delete cloud_height_property_;
  delete cloud_size_property_;
  delete cloud_color_property_;
  delete animation_speed_property_;
  delete persistence_time_property_;
  delete position_x_offset_property_;
  delete position_y_offset_property_;
  delete show_placeholder_property_;
}

void AttentionCloudDisplay::onInitialize()
{
  MessageFilterDisplay::onInitialize();
  createThinkingCloud();
}

void AttentionCloudDisplay::onEnable()
{
  MessageFilterDisplay::onEnable();
  if (cloud_node_) {
    cloud_node_->setVisible(true);
    RCLCPP_INFO(rclcpp::get_logger("AttentionCloudDisplay"), "Cloud node set to visible");
  }
}

void AttentionCloudDisplay::onDisable()
{
  if (cloud_node_) {
    cloud_node_->setVisible(false);
  }
  MessageFilterDisplay::onDisable();
}

void AttentionCloudDisplay::update(float wall_dt, float /*ros_dt*/)
{
  animateCloud(wall_dt);
}

void AttentionCloudDisplay::processMessage(const perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  // Handle frame transforms like other plugins
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  if (cloud_node_) {
    // Apply position offsets and height
    float x_offset = position_x_offset_property_->getValue().toFloat();
    float y_offset = position_y_offset_property_->getValue().toFloat();
    float cloud_height = cloud_height_property_->getValue().toFloat();
    cloud_node_->setPosition(position.x + x_offset, position.y + y_offset, position.z + cloud_height);
    
  }

  // TEMPORARY: Always show cloud for testing mesh rendering
  target_opacity_ = 1.0f;
  has_pedestrians_detected_ = true;
  persistence_timer_ = 0.0f;
  
  // Check for pedestrians in front of vehicle
  bool pedestrians_detected = hasPedestriansInFront(*msg);
  // 
  // Update persistence logic for smoother behavior
  if (pedestrians_detected) {
    persistence_timer_ = 0.0f; // Reset timer when pedestrians are detected
    if (!has_pedestrians_detected_) {
      has_pedestrians_detected_ = true;
      target_opacity_ = 1.0f;
    }
  }
  // Note: Timer increment happens in animateCloud, fade out decision made there
}

bool AttentionCloudDisplay::hasPedestriansInFront(const perception_msgs::msg::ObjectList& objects)
{
  float detection_range = detection_range_property_->getValue().toFloat();
  
  for (const auto& object : objects.objects) {
    // Get the classification with highest probability
    auto classification = perception_msgs::object_access::getClassWithHighestProbability(object);
    
    // Check if object is a pedestrian (PEDESTRIAN = 1 from ObjectClassification.msg)
    if (classification.type == perception_msgs::msg::ObjectClassification::PEDESTRIAN) {
      // Get position using utility functions
      float x = perception_msgs::object_access::getX(object);
      float y = perception_msgs::object_access::getY(object);
      
      // Check if pedestrian is in front (positive X) within range
      if (x > 0 && x <= detection_range && std::abs(y) <= detection_range/2) {
        return true;
      }
    }
  }
  return false;
}

void AttentionCloudDisplay::createThinkingCloud()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();
  
  // Create main cloud scene node
  cloud_node_ = scene_manager->getRootSceneNode()->createChildSceneNode();
  
  // Load the cloud mesh
  loadCloudMesh();
  
  // Position cloud above vehicle
  updateCloudProperties();
}

void AttentionCloudDisplay::loadCloudMesh()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();
  
  try {
    std::string package = "package://perception_msgs_rviz_plugins/meshes/bubble.obj";
    Ogre::MeshPtr mesh = rviz_rendering::loadMeshFromResource(package);  
    
    if (mesh) {
      std::string mesh_name = "cloud_mesh_" + std::to_string(reinterpret_cast<uintptr_t>(this));
      cloud_entity_ = scene_manager->createEntity(mesh_name, mesh);

      cloud_node_->attachObject(cloud_entity_);
      cloud_node_->setVisible(true);
      
      RCLCPP_INFO(rclcpp::get_logger("AttentionCloudDisplay"), "Successfully loaded cloud mesh using its default material.");
      
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("AttentionCloudDisplay"), "Failed to load cloud mesh: mesh is null");
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("AttentionCloudDisplay"), "Failed to load cloud mesh: %s", e.what());
  }
}


void AttentionCloudDisplay::updateCloudProperties()
{
  if (!cloud_node_ || !cloud_entity_) {
    return;
  }
  
  float cloud_height = cloud_height_property_->getValue().toFloat();
  float cloud_size = cloud_size_property_->getValue().toFloat();
  QColor cloud_color = cloud_color_property_->getColor();
  
  // Position cloud above vehicle
  cloud_node_->setPosition(0, 0, cloud_height);
  
  // Set mesh scale based on cloud size
  cloud_node_->setScale(cloud_size, cloud_size, cloud_size);
  
  // Update material color and opacity
  if (cloud_material_) {
    Ogre::Pass* pass = cloud_material_->getTechnique(0)->getPass(0);
    pass->setDiffuse(cloud_color.redF(), cloud_color.greenF(), cloud_color.blueF(), current_opacity_);
    pass->setAmbient(cloud_color.redF() * 0.8f, cloud_color.greenF() * 0.8f, cloud_color.blueF() * 0.8f);
  }
}


void AttentionCloudDisplay::animateCloud(float dt)
{
  if (!cloud_material_) {
    return;
  }
  
  // Update persistence timer - always increment when cloud should be visible
  if (has_pedestrians_detected_) {
    persistence_timer_ += dt;
    
    // Check if persistence time has expired - trigger fade out
    float persistence_time = persistence_time_property_->getValue().toFloat();
    if (persistence_timer_ >= persistence_time && target_opacity_ > 0.0f) {
      has_pedestrians_detected_ = false;
      target_opacity_ = 0.0f;
    }
  }
  
  // Different animation speeds for fade in vs fade out
  float fade_in_speed = 2.0f;   // 0.5 seconds to fade in (1.0 / 0.5 = 2.0)
  float fade_out_speed = 0.33f; // 3 seconds to fade out (1.0 / 3.0 = 0.33)
  
  // Smooth opacity transition with different speeds
  if (std::abs(current_opacity_ - target_opacity_) > 0.01f) {
    float opacity_change;
    if (current_opacity_ < target_opacity_) {
      // Fading in - use fast speed
      opacity_change = fade_in_speed * dt;
      current_opacity_ = std::min(target_opacity_, current_opacity_ + opacity_change);
    } else {
      // Fading out - use slow speed
      opacity_change = fade_out_speed * dt;
      current_opacity_ = std::max(target_opacity_, current_opacity_ - opacity_change);
    }
    
    // Update geometry when opacity changes
    updateCloudProperties();
  }
  
  // Add gentle floating animation with position offsets
  animation_time_ += dt;
  float base_height = cloud_height_property_->getValue().toFloat();
  float x_offset = position_x_offset_property_->getValue().toFloat();
  float y_offset = position_y_offset_property_->getValue().toFloat();
  
  if (cloud_node_) {
    cloud_node_->setPosition(x_offset, y_offset, base_height);
  }
  
  // Show/hide cloud based on opacity
  bool should_be_visible = current_opacity_ > 0.01f;
  if (cloud_visible_ != should_be_visible) {
    cloud_visible_ = should_be_visible;
    if (cloud_node_) {
      cloud_node_->setVisible(cloud_visible_);
    }
  }
}

void AttentionCloudDisplay::updateCloudVisibility()
{
  if (cloud_node_) {
    cloud_node_->setVisible(isEnabled());
  }
}

void AttentionCloudDisplay::destroyCloud()
{
  if (cloud_entity_) {
    if (cloud_node_) {
      cloud_node_->detachObject(cloud_entity_);
    }
    if (context_ && context_->getSceneManager()) {
      context_->getSceneManager()->destroyEntity(cloud_entity_);
    }
    cloud_entity_ = nullptr;
  }
  
  if (cloud_node_) {
    if (context_ && context_->getSceneManager()) {
      context_->getSceneManager()->destroySceneNode(cloud_node_);
    }
    cloud_node_ = nullptr;
  }
  
  if (cloud_material_) {
    cloud_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(cloud_material_->getName());
    cloud_material_.reset();
  }
}


}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionCloudDisplay, rviz_common::Display)