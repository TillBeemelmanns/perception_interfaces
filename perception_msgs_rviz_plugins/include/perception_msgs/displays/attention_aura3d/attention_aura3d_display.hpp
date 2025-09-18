#ifndef PERCEPTION_MSGS__DISPLAYS__ATTENTION_AURA3D__ATTENTION_AURA3D_DISPLAY_HPP_
#define PERCEPTION_MSGS__DISPLAYS__ATTENTION_AURA3D__ATTENTION_AURA3D_DISPLAY_HPP_

#include <memory>
#include <vector>
#include <string>

#include <OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreMaterial.h>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include <perception_msgs/msg/object_list.hpp>
#include "perception_msgs_utils/object_access.hpp"

namespace perception_msgs
{
namespace displays
{

struct DirectionSector3D {
  float attention_level;    // 0.0 to 1.0 - normalized attention strength
  int object_count;        // Number of objects in this direction
  float target_opacity;    // Target opacity for smooth transitions
  float current_opacity;   // Current opacity for smooth animations
  bool has_objects;        // Whether this direction has any objects
};

class AttentionAura3DDisplay : public rviz_common::MessageFilterDisplay<perception_msgs::msg::ObjectList>
{
  Q_OBJECT

public:
  AttentionAura3DDisplay();
  ~AttentionAura3DDisplay() override;

protected:
  void onInitialize() override;
  void reset() override;

private:
  void processMessage(const perception_msgs::msg::ObjectList::ConstSharedPtr msg) override;
  void updateAura();
  void createAuraElements();
  void destroyAuraElements();
  
  // Helper functions
  int getSectorIndex(float angle_rad) const;
  void analyzeSectors(const perception_msgs::msg::ObjectList::ConstSharedPtr msg);
  void createTrapezoidalElement(int sector_index, float opacity);
  void updateElementOpacity(int sector_index, float opacity);
  
  // Properties
  rviz_common::properties::FloatProperty* radius_property_;
  // Height property removed - elements are always flat
  rviz_common::properties::FloatProperty* thickness_property_;
  rviz_common::properties::ColorProperty* color_property_;
  rviz_common::properties::ColorProperty* border_color_property_;
  rviz_common::properties::FloatProperty* alpha_property_;
  rviz_common::properties::StringProperty* frame_property_;
  rviz_common::properties::BoolProperty* show_labels_property_;
  
  // 3D Objects
  std::vector<Ogre::SceneNode*> sector_nodes_;
  std::vector<Ogre::ManualObject*> sector_objects_;
  std::vector<Ogre::MaterialPtr> sector_materials_;
  
  // Data
  std::vector<DirectionSector3D> sectors_;
  static constexpr int NUM_SECTORS = 8;
  static constexpr float SECTOR_ANGLE = 2.0f * M_PI / NUM_SECTORS;
  
  // Animation
  void updateAnimations();
  static int material_counter_;

private Q_SLOTS:
  void updateAuraProperties();
};

} // namespace displays
} // namespace perception_msgs

#endif // PERCEPTION_MSGS__DISPLAYS__ATTENTION_AURA3D__ATTENTION_AURA3D_DISPLAY_HPP_