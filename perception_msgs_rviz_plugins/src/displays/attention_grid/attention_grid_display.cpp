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

#include "perception_msgs/displays/attention_grid/attention_grid_display.hpp"
#include "perception_msgs/displays/attention_grid/attention_swatch.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/time.hpp"
#include "rviz_rendering/material_manager.hpp"

namespace perception_msgs {
namespace displays {

AttentionGridDisplay::AttentionGridDisplay()
: loaded_(false),
  resolution_(0.0f),
  width_(0),
  height_(0)
{
  connect(this, SIGNAL(mapUpdated()), this, SLOT(showMap()));

  colormap_property_ = new rviz_common::properties::EnumProperty(
    "Colormap", "Jet",
    "Colormap for visualizing occupancy values.",
    this, SLOT(updatePalette()));
  colormap_property_->addOption("Jet", 0);
  colormap_property_->addOption("Hot", 1);
  colormap_property_->addOption("Magma", 2);
  colormap_property_->addOption("Viridis", 3);
  colormap_property_->addOption("Plasma", 4);
  colormap_property_->addOption("Grayscale", 5);

  use_alpha_transparency_ = new rviz_common::properties::BoolProperty(
    "Use Alpha Transparency", true,
    "Make low probability cells transparent based on their values.",
    this, SLOT(updatePalette()));

  alpha_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Alpha Threshold", 0.2f,
    "Probability threshold below which cells become transparent.",
    this, SLOT(updatePalette()));
  alpha_threshold_property_->setMin(0.0f);
  alpha_threshold_property_->setMax(1.0f);

  colormap_max_value_property_ = new rviz_common::properties::FloatProperty(
    "Colormap Max Value", 1.0f,
    "Maximum value for colormap scaling.",
    this, SLOT(updatePalette()));
  colormap_max_value_property_->setMin(0.001f);

  draw_under_property_ = new rviz_common::properties::BoolProperty(
    "Draw Behind", false,
    "Render the grid behind all other objects.",
    this, SLOT(updateDrawUnder()));

  resolution_property_ = new rviz_common::properties::FloatProperty(
    "Resolution", 0.0f,
    "Resolution of the grid (not editable).", this);
  resolution_property_->setReadOnly(true);

  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 0,
    "Width of the grid in cells (not editable).", this);
  width_property_->setReadOnly(true);

  height_property_ = new rviz_common::properties::IntProperty(
    "Height", 0,
    "Height of the grid in cells (not editable).", this);
  height_property_->setReadOnly(true);

  position_property_ = new rviz_common::properties::VectorProperty(
    "Position", Ogre::Vector3::ZERO,
    "Position of the grid origin (not editable).", this);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz_common::properties::QuaternionProperty(
    "Orientation", Ogre::Quaternion::IDENTITY, 
    "Orientation of the grid (not editable).", this);
  orientation_property_->setReadOnly(true);
}

AttentionGridDisplay::~AttentionGridDisplay()
{
  unsubscribe();
  clear();
  
  delete colormap_property_;
  delete colormap_max_value_property_;
  delete use_alpha_transparency_;
  delete alpha_threshold_property_;
  delete draw_under_property_;
  delete resolution_property_;
  delete width_property_;
  delete height_property_;
  delete position_property_;
  delete orientation_property_;
}

void AttentionGridDisplay::onInitialize()
{
  MFDClass::onInitialize();
  
  // Create initial palette textures
  updatePalette();
}


void AttentionGridDisplay::updateDrawUnder()
{
  bool draw_under = draw_under_property_->getBool();

  if (alpha_property_->getFloat() >= rviz_rendering::unit_alpha_threshold) {
    for (const auto & swatch : swatches_) {
      swatch->setDepthWriteEnabled(!draw_under);
    }
  }

  uint8_t group = draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN;
  for (const auto & swatch : swatches_) {
    swatch->setRenderQueueGroup(group);
  }
}

void AttentionGridDisplay::updateAlphaThreshold()
{
  updatePalette();
}

void AttentionGridDisplay::clear()
{
  if (isEnabled()) {
    setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No occupancy grid received");
  }

  if (!loaded_) {
    return;
  }

  swatches_.clear();
  height_ = 0;
  width_ = 0;
  resolution_ = 0.0f;
  loaded_ = false;
}

void AttentionGridDisplay::reset()
{
  MFDClass::reset();
  clear();
}

void AttentionGridDisplay::onEnable()
{
  MFDClass::onEnable();
  setStatus(rviz_common::properties::StatusProperty::Warn, "Message", "No occupancy grid received");
}

bool validateFloats(const nav_msgs::msg::OccupancyGrid & msg)
{
  return rviz_common::validateFloats(msg.info.resolution) &&
         rviz_common::validateFloats(msg.info.origin);
}

void AttentionGridDisplay::processMessage(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  current_map_ = *msg;
  loaded_ = true;
  // Updated via signal in case ROS spinner is in a different thread
  Q_EMIT mapUpdated();
}

void AttentionGridDisplay::showMap()
{
  if (current_map_.data.empty()) {
    return;
  }

  if (!validateFloats(current_map_)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Grid",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;

  if (width * height == 0) {
    std::string message =
      "Grid is zero-sized (" + std::to_string(width) + "x" + std::to_string(height) + ")";
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Grid", QString::fromStdString(message));
    return;
  }

  if (width * height != current_map_.data.size()) {
    std::string message =
      "Data size doesn't match width*height: width = " + std::to_string(width) + ", height = " +
      std::to_string(height) + ", data size = " + std::to_string(current_map_.data.size());
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Grid", QString::fromStdString(message));
    return;
  }

  setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Occupancy grid received");
  showValidMap();
}

void AttentionGridDisplay::showValidMap()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;
  float resolution = current_map_.info.resolution;

  resetSwatchesIfNecessary(width, height, resolution);

  frame_ = current_map_.header.frame_id;
  if (frame_.empty()) {
    frame_ = "/map";
  }

  updateSwatches();
  
  setStatus(rviz_common::properties::StatusProperty::Ok, "Grid", "Grid OK");
  updatePalette();

  resolution_property_->setValue(resolution);
  width_property_->setValue(static_cast<int>(width));
  height_property_->setValue(static_cast<int>(height));

  position_property_->setVector(rviz_common::pointMsgToOgre(current_map_.info.origin.position));
  orientation_property_->setQuaternion(
    rviz_common::quaternionMsgToOgre(current_map_.info.origin.orientation));

  transformMap();
  updateDrawUnder();
  context_->queueRender();
}

void AttentionGridDisplay::resetSwatchesIfNecessary(size_t width, size_t height, float resolution)
{
  if (width != width_ || height != height_ || resolution_ != resolution) {
    createSwatches();
    width_ = width;
    height_ = height;
    resolution_ = resolution;
  }
}

void AttentionGridDisplay::createSwatches()
{
  size_t width = current_map_.info.width;
  size_t height = current_map_.info.height;
  float resolution = current_map_.info.resolution;

  swatches_.clear();
  
  // For simplicity, create one large swatch. In production, you might want
  // to split large maps into multiple swatches like the original implementation
  try {
    swatches_.push_back(
      std::make_shared<AttentionSwatch>(
        scene_manager_,
        scene_node_,
        0, 0, width, height, resolution,
        draw_under_property_->getBool()));
    
    updateDrawUnder();
  } catch (const std::exception& e) {
    RVIZ_COMMON_LOG_ERROR_STREAM("Failed to create attention swatch: " << e.what());
    swatches_.clear();
  }
}

void AttentionGridDisplay::updateSwatches() const
{
  for (const auto & swatch : swatches_) {
    swatch->updateData(current_map_);

    Ogre::Pass * pass = swatch->getTechniquePass();
    if (!pass) continue;
    
    Ogre::TextureUnitState * tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 0) {
      tex_unit = pass->getTextureUnitState(0);
    } else {
      tex_unit = pass->createTextureUnitState();
    }

    tex_unit->setTextureName(swatch->getTextureName());
    tex_unit->setTextureFiltering(Ogre::TFO_NONE);
    swatch->setVisible(true);
    swatch->resetOldTexture();
  }
}

void AttentionGridDisplay::updatePalette()
{
  if (swatches_.empty()) {
    return;
  }

  int colormap_index = colormap_property_->getOptionInt();
  float alpha_threshold = alpha_threshold_property_->getFloat();
  float max_value = colormap_max_value_property_->getFloat();
  bool use_alpha = use_alpha_transparency_->getBool();
  
  std::string colormap_name;
  switch (colormap_index) {
    case 0: colormap_name = "jet"; break;
    case 1: colormap_name = "hot"; break;
    case 2: colormap_name = "magma"; break;
    case 3: colormap_name = "viridis"; break;
    case 4: colormap_name = "plasma"; break;
    case 5: default: colormap_name = "grayscale"; break;
  }
  
  std::vector<unsigned char> palette_bytes = createAdvancedPalette(
    colormap_name, use_alpha ? alpha_threshold : 0.0f, max_value);
  
  // Create palette texture
  Ogre::DataStreamPtr palette_stream;
  palette_stream.reset(new Ogre::MemoryDataStream(palette_bytes.data(), 256 * 4));
  
  static int palette_tex_count = 0;
  std::string tex_name = "AttentionGridPaletteTexture" + std::to_string(palette_tex_count++);
  Ogre::TexturePtr palette_texture = Ogre::TextureManager::getSingleton().loadRawData(
    tex_name, "rviz_rendering", palette_stream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);

  // Apply palette to swatches
  for (const auto & swatch : swatches_) {
    Ogre::Pass * pass = swatch->getTechniquePass();
    if (!pass) continue;
    
    Ogre::TextureUnitState * palette_tex_unit = nullptr;
    if (pass->getNumTextureUnitStates() > 1) {
      palette_tex_unit = pass->getTextureUnitState(1);
    } else {
      palette_tex_unit = pass->createTextureUnitState();
    }
    
    palette_tex_unit->setTexture(palette_texture);
    palette_tex_unit->setTextureFiltering(Ogre::TFO_NONE);
  }

  updateAlpha();
  updateDrawUnder();
}

void AttentionGridDisplay::transformMap()
{
  if (!loaded_) {
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->transform(
      frame_, rclcpp::Time(current_map_.header.stamp, RCL_ROS_TIME),
      current_map_.info.origin, position, orientation) &&
    !context_->getFrameManager()->transform(
      frame_, rclcpp::Time(0, 0, context_->getClock()->get_clock_type()),
      current_map_.info.origin, position, orientation))
  {
    setMissingTransformToFixedFrame(frame_);
    scene_node_->setVisible(false);
  } else {
    setTransformOk();
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
    scene_node_->setVisible(true);
  }
}

void AttentionGridDisplay::fixedFrameChanged()
{
  transformMap();
}

void AttentionGridDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  transformMap();
}

// Advanced colormap creation functions
std::vector<unsigned char> AttentionGridDisplay::createAdvancedPalette(
  const std::string& colormap_name, float alpha_threshold, float max_value)
{
  if (colormap_name == "jet") {
    return createJetPalette(alpha_threshold);
  } else if (colormap_name == "hot") {
    return createHotPalette(alpha_threshold);
  } else if (colormap_name == "magma") {
    return createMagmaPalette(alpha_threshold);
  } else if (colormap_name == "viridis") {
    return createViridsPalette(alpha_threshold);
  } else if (colormap_name == "plasma") {
    return createPlasmaPalette(alpha_threshold);
  } else {
    // Default grayscale
    std::vector<unsigned char> palette(256 * 4);
    for (int i = 0; i < 256; ++i) {
      unsigned char gray = i;
      unsigned char alpha = 255;
      
      // Apply alpha transparency for low values
      if (alpha_threshold > 0.0f && i < static_cast<int>(alpha_threshold * 255)) {
        alpha = static_cast<unsigned char>((i / (alpha_threshold * 255)) * 255);
      }
      
      palette[i * 4 + 0] = gray;  // R
      palette[i * 4 + 1] = gray;  // G
      palette[i * 4 + 2] = gray;  // B
      palette[i * 4 + 3] = alpha; // A
    }
    return palette;
  }
}

std::vector<unsigned char> AttentionGridDisplay::createJetPalette(float alpha_threshold)
{
  std::vector<unsigned char> palette(256 * 4);
  
  for (int i = 0; i < 256; ++i) {
    float x = i / 255.0f;
    unsigned char r, g, b, a = 255;
    
    // Jet colormap calculation
    if (x < 0.125f) {
      r = 0;
      g = 0;
      b = static_cast<unsigned char>(255.0f * (0.5f + x * 4));
    } else if (x < 0.375f) {
      r = 0;
      g = static_cast<unsigned char>(255.0f * (x - 0.125f) * 4);
      b = 255;
    } else if (x < 0.625f) {
      r = static_cast<unsigned char>(255.0f * (x - 0.375f) * 4);
      g = 255;
      b = static_cast<unsigned char>(255.0f * (0.625f - x) * 4);
    } else if (x < 0.875f) {
      r = 255;
      g = static_cast<unsigned char>(255.0f * (0.875f - x) * 4);
      b = 0;
    } else {
      r = static_cast<unsigned char>(255.0f * (1.125f - x) * 4);
      g = 0;
      b = 0;
    }
    
    // Apply alpha transparency for low values
    if (alpha_threshold > 0.0f && x < alpha_threshold) {
      a = static_cast<unsigned char>((x / alpha_threshold) * 255.0f);
    }
    
    palette[i * 4 + 0] = r;
    palette[i * 4 + 1] = g;
    palette[i * 4 + 2] = b;
    palette[i * 4 + 3] = a;
  }
  
  return palette;
}

std::vector<unsigned char> AttentionGridDisplay::createHotPalette(float alpha_threshold)
{
  std::vector<unsigned char> palette(256 * 4);
  
  for (int i = 0; i < 256; ++i) {
    float x = i / 255.0f;
    unsigned char r, g, b, a = 255;
    
    // Hot colormap calculation
    if (x < 0.4f) {
      r = static_cast<unsigned char>(255.0f * x / 0.4f);
      g = 0;
      b = 0;
    } else if (x < 0.8f) {
      r = 255;
      g = static_cast<unsigned char>(255.0f * (x - 0.4f) / 0.4f);
      b = 0;
    } else {
      r = 255;
      g = 255;
      b = static_cast<unsigned char>(255.0f * (x - 0.8f) / 0.2f);
    }
    
    // Apply alpha transparency for low values
    if (alpha_threshold > 0.0f && x < alpha_threshold) {
      a = static_cast<unsigned char>((x / alpha_threshold) * 255.0f);
    }
    
    palette[i * 4 + 0] = r;
    palette[i * 4 + 1] = g;
    palette[i * 4 + 2] = b;
    palette[i * 4 + 3] = a;
  }
  
  return palette;
}

std::vector<unsigned char> AttentionGridDisplay::createMagmaPalette(float alpha_threshold)
{
  std::vector<unsigned char> palette(256 * 4);
  
  // Magma colormap approximation
  for (int i = 0; i < 256; ++i) {
    float x = i / 255.0f;
    unsigned char r, g, b, a = 255;
    
    // Simplified magma colormap
    r = static_cast<unsigned char>(255.0f * std::min(1.0f, std::max(0.0f, 1.5f * x - 0.2f)));
    g = static_cast<unsigned char>(255.0f * std::min(1.0f, std::max(0.0f, 2.0f * x - 0.8f)));
    b = static_cast<unsigned char>(255.0f * std::min(1.0f, std::max(0.0f, 1.2f * x)));
    
    // Apply alpha transparency for low values
    if (alpha_threshold > 0.0f && x < alpha_threshold) {
      a = static_cast<unsigned char>((x / alpha_threshold) * 255.0f);
    }
    
    palette[i * 4 + 0] = r;
    palette[i * 4 + 1] = g;
    palette[i * 4 + 2] = b;
    palette[i * 4 + 3] = a;
  }
  
  return palette;
}

std::vector<unsigned char> AttentionGridDisplay::createViridsPalette(float alpha_threshold)
{
  std::vector<unsigned char> palette(256 * 4);
  
  // Viridis colormap approximation
  for (int i = 0; i < 256; ++i) {
    float x = i / 255.0f;
    unsigned char r, g, b, a = 255;
    
    // Simplified viridis colormap
    r = static_cast<unsigned char>(255.0f * (0.1f + 0.6f * x));
    g = static_cast<unsigned char>(255.0f * x);
    b = static_cast<unsigned char>(255.0f * (0.9f - 0.3f * x));
    
    // Apply alpha transparency for low values
    if (alpha_threshold > 0.0f && x < alpha_threshold) {
      a = static_cast<unsigned char>((x / alpha_threshold) * 255.0f);
    }
    
    palette[i * 4 + 0] = r;
    palette[i * 4 + 1] = g;
    palette[i * 4 + 2] = b;
    palette[i * 4 + 3] = a;
  }
  
  return palette;
}

std::vector<unsigned char> AttentionGridDisplay::createPlasmaPalette(float alpha_threshold)
{
  std::vector<unsigned char> palette(256 * 4);
  
  // Plasma colormap approximation
  for (int i = 0; i < 256; ++i) {
    float x = i / 255.0f;
    unsigned char r, g, b, a = 255;
    
    // Simplified plasma colormap
    r = static_cast<unsigned char>(255.0f * (0.8f + 0.2f * std::sin(3.14159f * x)));
    g = static_cast<unsigned char>(255.0f * (0.3f + 0.7f * x));
    b = static_cast<unsigned char>(255.0f * std::max(0.0f, 1.0f - 2.0f * x));
    
    // Apply alpha transparency for low values
    if (alpha_threshold > 0.0f && x < alpha_threshold) {
      a = static_cast<unsigned char>((x / alpha_threshold) * 255.0f);
    }
    
    palette[i * 4 + 0] = r;
    palette[i * 4 + 1] = g;
    palette[i * 4 + 2] = b;
    palette[i * 4 + 3] = a;
  }
  
  return palette;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionGridDisplay, rviz_common::Display)