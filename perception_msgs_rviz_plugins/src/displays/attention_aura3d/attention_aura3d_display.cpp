#include <perception_msgs/displays/attention_aura3d/attention_aura3d_display.hpp>

#include <cmath>
#include <algorithm>
#include <sstream>

#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreTextureUnitState.h>
#include <OgreColourValue.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/logging.hpp>

namespace perception_msgs
{
namespace displays
{

int AttentionAura3DDisplay::material_counter_ = 0;

AttentionAura3DDisplay::AttentionAura3DDisplay()
: MessageFilterDisplay<perception_msgs::msg::ObjectList>(),
  sectors_(NUM_SECTORS)
{
  // Initialize properties
  radius_property_ = new rviz_common::properties::FloatProperty(
    "Radius", 10.0f,
    "Radius of the flat aura elements from vehicle center",
    this, SLOT(updateAuraProperties()));
  radius_property_->setMin(2.0f);
  radius_property_->setMax(50.0f);

  // Height property removed - elements are always flat on ground

  thickness_property_ = new rviz_common::properties::FloatProperty(
    "Thickness", 10.0f,
    "Radial thickness of the flat trapezoidal elements",
    this, SLOT(updateAuraProperties()));
  thickness_property_->setMin(0.5f);
  thickness_property_->setMax(15.0f);

  color_property_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 255, 255),
    "Base color of the aura elements",
    this, SLOT(updateAuraProperties()));

  border_color_property_ = new rviz_common::properties::ColorProperty(
    "Border Color", QColor(255, 255, 255),
    "Color of the element borders",
    this, SLOT(updateAuraProperties()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Alpha", 1.0f,
    "Base transparency of the aura elements",
    this, SLOT(updateAuraProperties()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);

  frame_property_ = new rviz_common::properties::StringProperty(
    "Reference Frame", "base_link",
    "Reference frame for the aura visualization",
    this, SLOT(updateAuraProperties()));

  show_labels_property_ = new rviz_common::properties::BoolProperty(
    "Show Labels", false,
    "Show directional labels (N, NE, E, etc.)",
    this, SLOT(updateAuraProperties()));

  // Initialize sectors with base visibility for debugging
  for (auto& sector : sectors_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.target_opacity = 0.2f; // Start dim - will brighten with objects
    sector.current_opacity = 0.2f;
    sector.has_objects = false;
  }
}

AttentionAura3DDisplay::~AttentionAura3DDisplay()
{
  destroyAuraElements();
}

void AttentionAura3DDisplay::onInitialize()
{
  MessageFilterDisplay::onInitialize();
  createAuraElements();
}

void AttentionAura3DDisplay::reset()
{
  MessageFilterDisplay::reset();
  destroyAuraElements();
  createAuraElements();
}

void AttentionAura3DDisplay::processMessage(const perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  // Handle frame transforms like attention_vector does
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    setMissingTransformToFixedFrame(msg->header.frame_id);
    return;
  }
  setTransformOk();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  analyzeSectors(msg);
  updateAura();
}

void AttentionAura3DDisplay::analyzeSectors(const perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  // Reset sectors
  for (auto& sector : sectors_) {
    sector.attention_level = 0.0f;
    sector.object_count = 0;
    sector.has_objects = false;
  }

  if (msg->objects.empty()) {
    // If no objects, set all sectors to low visibility
    for (auto& sector : sectors_) {
      sector.target_opacity = 0.2f; // Low baseline when no objects
    }
    return;
  }

  // Analyze objects and distribute attention
  float total_attention = 0.0f;
  std::vector<float> sector_attention(NUM_SECTORS, 0.0f);

  for (const auto& object : msg->objects) {
    // Calculate angle from ego to object
    float dx = static_cast<float>(perception_msgs::object_access::getX(object));
    float dy = static_cast<float>(perception_msgs::object_access::getY(object));
    float angle = std::atan2(dy, dx);
    
    int sector_idx = getSectorIndex(angle);
    
    // Calculate attention based on distance (closer objects get more attention)
    float distance = std::sqrt(dx * dx + dy * dy);
    float attention_weight = 1.0f / (1.0f + distance * 0.1f); // Inverse distance weighting
    
    sector_attention[sector_idx] += attention_weight;
    sectors_[sector_idx].object_count++;
    sectors_[sector_idx].has_objects = true;
    
    total_attention += attention_weight;
  }

  // Normalize attention levels and set dynamic target opacities
  if (total_attention > 0.0f) {
    for (int i = 0; i < NUM_SECTORS; ++i) {
      sectors_[i].attention_level = sector_attention[i] / total_attention;
      
      if (sectors_[i].has_objects) {
        // Sectors with objects: bright and attention-based (0.8 to 1.0)
        sectors_[i].target_opacity = std::max(0.8f, std::min(1.0f, sectors_[i].attention_level * 3.0f));
      } else {
        // Sectors without objects: dim baseline
        sectors_[i].target_opacity = 0.2f;
      }
    }
  } else {
    // Fallback when no attention calculated
    for (int i = 0; i < NUM_SECTORS; ++i) {
      sectors_[i].target_opacity = sectors_[i].has_objects ? 0.7f : 0.2f;
    }
  }
}

int AttentionAura3DDisplay::getSectorIndex(float angle_rad) const
{
  // Rotate by π/2 (90 degrees) so that sector 0 = North (forward direction)
  // Then rotate 45 degrees clockwise (subtract π/4) to align directions properly
  float adjusted_angle = angle_rad + (M_PI / 2.0f) + (M_PI / 8.0f) - (M_PI / 4.0f);
  
  // Normalize to [0, 2π)
  while (adjusted_angle < 0) adjusted_angle += 2.0f * M_PI;
  while (adjusted_angle >= 2.0f * M_PI) adjusted_angle -= 2.0f * M_PI;
  
  int sector = static_cast<int>(adjusted_angle / SECTOR_ANGLE);
  return std::clamp(sector, 0, NUM_SECTORS - 1);
}

void AttentionAura3DDisplay::createAuraElements()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  destroyAuraElements();

  Ogre::SceneManager* scene_manager = context_->getSceneManager();
  
  for (int i = 0; i < NUM_SECTORS; ++i) {
    // Create scene node
    std::string node_name = "attention_aura3d_node_" + std::to_string(material_counter_) + "_" + std::to_string(i);
    Ogre::SceneNode* node = scene_manager->getRootSceneNode()->createChildSceneNode(node_name);
    sector_nodes_.push_back(node);

    // Create manual object for trapezoidal geometry
    std::string object_name = "attention_aura3d_object_" + std::to_string(material_counter_) + "_" + std::to_string(i);
    Ogre::ManualObject* manual_object = scene_manager->createManualObject(object_name);
    sector_objects_.push_back(manual_object);

    // Create material for this sector with vertex colors enabled
    std::string material_name = "AttentionAura3DMaterial_" + std::to_string(material_counter_++) + "_" + std::to_string(i);
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    Ogre::Pass* pass = material->getTechnique(0)->getPass(0);
    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthWriteEnabled(false);
    pass->setDepthCheckEnabled(true);
    pass->setLightingEnabled(false);
    pass->setCullingMode(Ogre::CULL_NONE);
    
    // Enable vertex colors - this is crucial!
    pass->setVertexColourTracking(Ogre::TVC_DIFFUSE);
    
    // Set base material to white so vertex colors show through properly
    pass->setDiffuse(1.0f, 1.0f, 1.0f, 1.0f);
    pass->setAmbient(0.8f, 0.8f, 0.8f); // Slightly less ambient for better contrast
    
    sector_materials_.push_back(material);

    node->attachObject(manual_object);
  }

  updateAura();
}

void AttentionAura3DDisplay::destroyAuraElements()
{
  if (!context_ || !context_->getSceneManager()) {
    return;
  }

  Ogre::SceneManager* scene_manager = context_->getSceneManager();

  // Destroy manual objects
  for (auto* obj : sector_objects_) {
    if (obj) {
      try {
        scene_manager->destroyManualObject(obj);
      } catch (...) {
        // Ignore destruction errors
      }
    }
  }
  sector_objects_.clear();

  // Destroy scene nodes
  for (auto* node : sector_nodes_) {
    if (node) {
      try {
        scene_manager->destroySceneNode(node);
      } catch (...) {
        // Ignore destruction errors
      }
    }
  }
  sector_nodes_.clear();

  // Clear materials
  sector_materials_.clear();
}

void AttentionAura3DDisplay::createTrapezoidalElement(int sector_index, float /*opacity*/)
{
  if (sector_index >= static_cast<int>(sector_objects_.size()) || !sector_objects_[sector_index]) {
    return;
  }

  Ogre::ManualObject* manual_object = sector_objects_[sector_index];
  manual_object->clear();

  // Calculate color and opacity for this element
  float radius = radius_property_->getValue().toFloat();
  float thickness = thickness_property_->getValue().toFloat();
  
  // Calculate sector angle and rotate 45 degrees clockwise
  float sector_angle = sector_index * SECTOR_ANGLE - (M_PI / 4.0f);
  
  // Get the pre-created material
  if (sector_index >= static_cast<int>(sector_materials_.size()) || !sector_materials_[sector_index]) {
    return; // No valid material
  }
  
  std::string material_name = sector_materials_[sector_index]->getName();
  
  // Calculate color based on current sector state
  QColor base_color = color_property_->getColor();
  float base_alpha = alpha_property_->getValue().toFloat();
  
  // Get current opacity from sector state (this is the key!)
  float current_opacity = (sector_index < static_cast<int>(sectors_.size())) ? 
    sectors_[sector_index].current_opacity : 0.3f;
  
  // Use the sector's dynamic opacity directly (no minimum override!)
  // This allows sectors to be dim (0.2) when no objects, bright (0.8-1.0) when objects detected
  
  // Apply small boost for visibility but preserve dynamic range
  float boosted_opacity = std::min(1.0f, current_opacity * 1.1f);
  
  Ogre::ColourValue vertex_color(
    base_color.redF(),
    base_color.greenF(),
    base_color.blueF(),
    base_alpha * boosted_opacity // Use boosted opacity
  );
  
  // Create flat trapezoidal sector shape (like HUD but on ground)
  manual_object->begin(material_name, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  // Use HUD's exact trapezoid geometry
  float inner_radius = radius - thickness * 0.5f;
  float outer_radius = radius + thickness * 0.5f;
  
  // Ensure valid radii  
  if (inner_radius <= 0) inner_radius = 1.0f;
  if (outer_radius <= inner_radius) outer_radius = inner_radius + 1.0f;
  
  // HUD's exact angular widths - narrow at inner edge, wider at outer edge
  float inner_half_angle = M_PI / 16.0f; // ±π/16 at inner edge (narrow)
  float outer_half_angle = M_PI / 8.0f;  // ±π/8 at outer edge (wide)
  
  // Create 4 vertices of the trapezoid (exactly like HUD)
  // Inner edge (narrow end)
  float inner_angle1 = sector_angle - inner_half_angle;
  float inner_angle2 = sector_angle + inner_half_angle;
  // Outer edge (wide end)
  float outer_angle1 = sector_angle - outer_half_angle;
  float outer_angle2 = sector_angle + outer_half_angle;
  
  // Add vertices in correct winding order (counter-clockwise) with colors
  // Vertex 0: Inner left
  manual_object->position(inner_radius * cos(inner_angle1), inner_radius * sin(inner_angle1), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Vertex 1: Inner right  
  manual_object->position(inner_radius * cos(inner_angle2), inner_radius * sin(inner_angle2), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Vertex 2: Outer right
  manual_object->position(outer_radius * cos(outer_angle2), outer_radius * sin(outer_angle2), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Vertex 3: Outer left
  manual_object->position(outer_radius * cos(outer_angle1), outer_radius * sin(outer_angle1), 0.1f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(vertex_color);
  
  // Create two triangles to form the trapezoid (counter-clockwise winding)
  manual_object->triangle(0, 1, 2); // First triangle
  manual_object->triangle(0, 2, 3); // Second triangle

  manual_object->end();

  // Add border rendering
  QColor border_color = border_color_property_->getColor();
  Ogre::ColourValue border_vertex_color(
    border_color.redF(),
    border_color.greenF(),
    border_color.blueF(),
    base_alpha * boosted_opacity // Same opacity as main element
  );

  // Create border as line strips - slightly elevated to avoid z-fighting
  manual_object->begin(material_name, Ogre::RenderOperation::OT_LINE_STRIP);
  
  // Border vertices (same positions but as line strip)
  manual_object->position(inner_radius * cos(inner_angle1), inner_radius * sin(inner_angle1), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  manual_object->position(inner_radius * cos(inner_angle2), inner_radius * sin(inner_angle2), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  manual_object->position(outer_radius * cos(outer_angle2), outer_radius * sin(outer_angle2), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  manual_object->position(outer_radius * cos(outer_angle1), outer_radius * sin(outer_angle1), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);
  
  // Close the border loop
  manual_object->position(inner_radius * cos(inner_angle1), inner_radius * sin(inner_angle1), 0.11f);
  manual_object->normal(0, 0, 1);
  manual_object->colour(border_vertex_color);

  manual_object->end();
}
void AttentionAura3DDisplay::updateElementOpacity(int /*sector_index*/, float /*opacity*/)
{
  // Opacity is now handled directly in createTrapezoidalElement via vertex colors
  // This function is kept for interface compatibility
}

void AttentionAura3DDisplay::updateAura()
{
  updateAnimations();
  
  for (int i = 0; i < NUM_SECTORS; ++i) {
    createTrapezoidalElement(i, sectors_[i].current_opacity);
    updateElementOpacity(i, sectors_[i].current_opacity);
  }
}

void AttentionAura3DDisplay::updateAnimations()
{
  // Fast animation like HUD (2.0f per second)
  const float animation_speed = 5.0f; // Even faster for more responsive feel
  const float dt = 1.0f / 60.0f; // Assume 60 FPS for consistent animation
  
  for (auto& sector : sectors_) {
    float diff = sector.target_opacity - sector.current_opacity;
    float step = animation_speed * dt;
    
    if (std::abs(diff) < step) {
      sector.current_opacity = sector.target_opacity;
    } else {
      sector.current_opacity += (diff > 0 ? step : -step);
    }
    
    // Clamp to valid range
    sector.current_opacity = std::clamp(sector.current_opacity, 0.0f, 1.0f);
  }
}

void AttentionAura3DDisplay::updateAuraProperties()
{
  updateAura();
}

} // namespace displays
} // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionAura3DDisplay, rviz_common::Display)