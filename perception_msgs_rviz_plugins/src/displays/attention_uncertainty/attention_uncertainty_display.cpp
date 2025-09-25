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

#include "perception_msgs/displays/attention_uncertainty/attention_uncertainty_display.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <QImage>
#include <QLinearGradient>
#include <QPainter>
#include <QFont>
#include <QFontMetrics>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/status_property.hpp"
#include "rviz_rendering/render_system.hpp"

namespace perception_msgs {
namespace displays {

namespace {
constexpr float kClampEpsilon = 1e-5f;
}

AttentionUncertaintyDisplay::AttentionUncertaintyDisplay()
: hud_width_property_(nullptr), hud_height_property_(nullptr),
  hud_left_property_(nullptr), hud_top_property_(nullptr),
  hud_alpha_property_(nullptr), bg_alpha_property_(nullptr),
  high_threshold_property_(nullptr), low_threshold_property_(nullptr),
  blink_threshold_property_(nullptr), blink_frequency_property_(nullptr),
  smoothing_alpha_property_(nullptr),
  high_color_property_(nullptr), mid_color_property_(nullptr),
  low_color_property_(nullptr), title_text_property_(nullptr),
  hud_width_(kDefaultWidth), hud_height_(kDefaultHeight),
  hud_left_(40), hud_top_(40), hud_alpha_(0.9f), bg_alpha_(0.35f),
  high_threshold_(0.75f), low_threshold_(0.45f), blink_threshold_(0.25f),
  blink_frequency_(2.0f), smoothing_alpha_(0.6f),
  high_color_(0, 220, 120), mid_color_(255, 200, 40),
  low_color_(255, 70, 70), title_text_(QStringLiteral("Perception Certainty")),
  smoothed_certainty_(0.0), have_certainty_(false),
  update_required_(false), blink_state_(false), blink_timer_(0.0),
  overlay_(nullptr), panel_(nullptr)
{
  hud_width_property_ = new rviz_common::properties::IntProperty(
    "HUD Width", hud_width_,
    "Width of the uncertainty overlay in pixels", this, SLOT(updateSize()));
  hud_width_property_->setMin(160);
  hud_width_property_->setMax(640);

  hud_height_property_ = new rviz_common::properties::IntProperty(
    "HUD Height", hud_height_,
    "Height of the uncertainty overlay in pixels", this, SLOT(updateSize()));
  hud_height_property_->setMin(120);
  hud_height_property_->setMax(480);

  hud_left_property_ = new rviz_common::properties::IntProperty(
    "Left", hud_left_,
    "Left screen position of the overlay (in pixels)", this, SLOT(updatePosition()));

  hud_top_property_ = new rviz_common::properties::IntProperty(
    "Top", hud_top_,
    "Top screen position of the overlay (in pixels)", this, SLOT(updatePosition()));

  hud_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Foreground Alpha", hud_alpha_,
    "Opacity applied to bar and frame elements (0 = transparent, 1 = opaque)",
    this, SLOT(updateTransparency()));
  hud_alpha_property_->setMin(0.0f);
  hud_alpha_property_->setMax(1.0f);

  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_,
    "Opacity of the background glass panel", this, SLOT(updateTransparency()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);

  high_threshold_property_ = new rviz_common::properties::FloatProperty(
    "High Certainty Threshold", high_threshold_,
    "Certainty above which the bar turns green", this, SLOT(updateThresholds()));
  high_threshold_property_->setMin(0.0f);
  high_threshold_property_->setMax(1.0f);

  low_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Medium Certainty Threshold", low_threshold_,
    "Certainty above this value is shown as amber", this, SLOT(updateThresholds()));
  low_threshold_property_->setMin(0.0f);
  low_threshold_property_->setMax(1.0f);

  blink_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Blink Threshold", blink_threshold_,
    "Certainty below this value triggers blinking red warning", this, SLOT(updateThresholds()));
  blink_threshold_property_->setMin(0.0f);
  blink_threshold_property_->setMax(1.0f);

  blink_frequency_property_ = new rviz_common::properties::FloatProperty(
    "Blink Frequency", blink_frequency_,
    "Blink frequency in Hz when certainty is critical", this, SLOT(updateThresholds()));
  blink_frequency_property_->setMin(0.2f);
  blink_frequency_property_->setMax(5.0f);

  smoothing_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Smoothing Factor", smoothing_alpha_,
    "Weight of previous certainty (0 = no smoothing, 0.9 = slow response)",
    this, SLOT(updateSmoothing()));
  smoothing_alpha_property_->setMin(0.0f);
  smoothing_alpha_property_->setMax(0.95f);

  high_color_property_ = new rviz_common::properties::ColorProperty(
    "High Certainty Color", high_color_,
    "Bar color when certainty is above the high threshold", this, SLOT(updateThresholds()));

  mid_color_property_ = new rviz_common::properties::ColorProperty(
    "Medium Certainty Color", mid_color_,
    "Bar color when certainty is between medium and high thresholds", this, SLOT(updateThresholds()));

  low_color_property_ = new rviz_common::properties::ColorProperty(
    "Low Certainty Color", low_color_,
    "Bar color when certainty is below the medium threshold", this, SLOT(updateThresholds()));

  title_text_property_ = new rviz_common::properties::StringProperty(
    "Title", title_text_,
    "Text displayed above the certainty bar (leave empty to hide)", this, SLOT(updateTitle()));

  updateTitle();
}

AttentionUncertaintyDisplay::~AttentionUncertaintyDisplay()
{
  destroyHUDOverlay();

  delete hud_width_property_;
  delete hud_height_property_;
  delete hud_left_property_;
  delete hud_top_property_;
  delete hud_alpha_property_;
  delete bg_alpha_property_;
  delete high_threshold_property_;
  delete low_threshold_property_;
  delete blink_threshold_property_;
  delete blink_frequency_property_;
  delete smoothing_alpha_property_;
  delete high_color_property_;
  delete mid_color_property_;
  delete low_color_property_;
  delete title_text_property_;
}

void AttentionUncertaintyDisplay::onInitialize()
{
  MFDClass::onInitialize();

  rviz_rendering::RenderSystem::get()->prepareOverlays(context_->getSceneManager());
  createHUDOverlay();
  {
    std::lock_guard<std::mutex> lock(hud_mutex_);
    updateHUD();
    update_required_ = false;
  }
}

void AttentionUncertaintyDisplay::onEnable()
{
  MFDClass::onEnable();
  if (overlay_) {
    overlay_->show();
  }
  update_required_ = true;
}

void AttentionUncertaintyDisplay::onDisable()
{
  MFDClass::onDisable();
  if (overlay_) {
    overlay_->hide();
  }
}

void AttentionUncertaintyDisplay::processMessage(perception_msgs::msg::ObjectList::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const double certainty = std::clamp(computeCertainty(*msg), 0.0, 1.0);

  std::lock_guard<std::mutex> lock(hud_mutex_);

  if (!have_certainty_) {
    smoothed_certainty_ = certainty;
    have_certainty_ = true;
  } else {
    const double clamped_factor = std::clamp(static_cast<double>(smoothing_alpha_), 0.0, 0.95);
    const double new_weight = 1.0 - clamped_factor;
    smoothed_certainty_ = clamped_factor * smoothed_certainty_ + new_weight * certainty;
  }

  std::ostringstream oss;
  oss << "Smoothed certainty: " << std::fixed << std::setprecision(2) << smoothed_certainty_;
  setStatus(rviz_common::properties::StatusProperty::Ok, "Certainty", oss.str().c_str());

  update_required_ = true;
}

void AttentionUncertaintyDisplay::update(float wall_dt, float /*ros_dt*/)
{
  if (!overlay_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(hud_mutex_);

    const bool critical = have_certainty_ && (smoothed_certainty_ <= static_cast<double>(blink_threshold_));

    if (critical && blink_frequency_ > kClampEpsilon) {
      blink_timer_ += wall_dt;
      const double half_period = 0.5 / static_cast<double>(blink_frequency_);
      if (blink_timer_ >= half_period) {
        blink_state_ = !blink_state_;
        blink_timer_ = 0.0;
        update_required_ = true;
      }
    } else {
      if (blink_state_) {
        blink_state_ = false;
        update_required_ = true;
      }
      blink_timer_ = 0.0;
    }

    if (update_required_) {
      updateHUD();
      update_required_ = false;
    }
  }
}

double AttentionUncertaintyDisplay::computeCertainty(const perception_msgs::msg::ObjectList& objects) const
{
  if (objects.objects.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  std::size_t count = 0;

  for (const auto& object : objects.objects) {
    double max_probability = 0.0;
    bool has_valid_classification = false;

    for (const auto& classification : object.state.classifications) {
      if (!std::isfinite(classification.probability)) {
        continue;
      }
      max_probability = std::max(max_probability, static_cast<double>(classification.probability));
      has_valid_classification = true;
    }

    if (!has_valid_classification) {
      // Treat missing classifications as uncertain
      max_probability = 0.0;
    }

    sum += std::clamp(max_probability, 0.0, 1.0);
    ++count;
  }

  if (count == 0) {
    return 0.0;
  }

  return sum / static_cast<double>(count);
}

void AttentionUncertaintyDisplay::createHUDOverlay()
{
  static int overlay_counter = 0;
  const std::string overlay_name = "AttentionUncertaintyOverlay" + std::to_string(overlay_counter);
  const std::string panel_name = "AttentionUncertaintyPanel" + std::to_string(overlay_counter);
  const std::string material_name = "AttentionUncertaintyMaterial" + std::to_string(overlay_counter);
  overlay_counter++;

  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (!overlay_mgr) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("AttentionUncertaintyDisplay"),
                        "Ogre OverlayManager not available");
    return;
  }

  overlay_ = overlay_mgr->create(overlay_name);

  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    overlay_mgr->createOverlayElement("Panel", panel_name));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    material_name, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_material_->setReceiveShadows(false);
  panel_material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);

  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
  overlay_->hide();
}

void AttentionUncertaintyDisplay::destroyHUDOverlay()
{
  Ogre::OverlayManager* overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (overlay_mgr) {
    if (panel_) {
      overlay_mgr->destroyOverlayElement(panel_);
      panel_ = nullptr;
    }
    if (overlay_) {
      overlay_mgr->destroy(overlay_);
      overlay_ = nullptr;
    }
  }

  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_->getName());
    texture_.setNull();
  }

  if (!panel_material_.isNull()) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
    panel_material_.setNull();
  }
}

void AttentionUncertaintyDisplay::updateHUD()
{
  if (!panel_ || panel_material_.isNull()) {
    return;
  }

  hud_width_ = hud_width_property_->getInt();
  hud_height_ = hud_height_property_->getInt();
  hud_left_ = hud_left_property_->getInt();
  hud_top_ = hud_top_property_->getInt();

  hud_alpha_ = hud_alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  high_threshold_ = std::clamp(high_threshold_property_->getFloat(), 0.0f, 1.0f);
  low_threshold_ = std::clamp(low_threshold_property_->getFloat(), 0.0f, high_threshold_);
  blink_threshold_ = std::clamp(blink_threshold_property_->getFloat(), 0.0f, low_threshold_);
  blink_frequency_ = std::max(blink_frequency_property_->getFloat(), kClampEpsilon);
  smoothing_alpha_ = std::clamp(smoothing_alpha_property_->getFloat(), 0.0f, 0.95f);
  high_color_ = high_color_property_->getColor();
  mid_color_ = mid_color_property_->getColor();
  low_color_ = low_color_property_->getColor();

  panel_->setPosition(static_cast<Ogre::Real>(hud_left_), static_cast<Ogre::Real>(hud_top_));
  panel_->setDimensions(static_cast<Ogre::Real>(hud_width_), static_cast<Ogre::Real>(hud_height_));

  if (hud_width_ <= 0) {
    hud_width_ = 1;
  }
  if (hud_height_ <= 0) {
    hud_height_ = 1;
  }

  const std::string texture_name = panel_material_->getName() + "Texture";

  const bool recreate_texture = texture_.isNull() ||
    texture_->getWidth() != static_cast<unsigned int>(hud_width_) ||
    texture_->getHeight() != static_cast<unsigned int>(hud_height_);

  if (recreate_texture) {
    if (!texture_.isNull()) {
      Ogre::TextureManager::getSingleton().remove(texture_->getName());
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }

    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      static_cast<Ogre::uint>(hud_width_),
      static_cast<Ogre::uint>(hud_height_),
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DYNAMIC);

    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    panel_material_->setCullingMode(Ogre::CULL_NONE);
  }

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer = texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pixel_box = pixel_buffer->getCurrentLock();

  QImage hud_image(static_cast<uchar*>(pixel_box.data), hud_width_, hud_height_, QImage::Format_ARGB32);

  QColor background(10, 16, 24, static_cast<int>(bg_alpha_ * 255));
  hud_image.fill(background.rgba());

  QPainter painter(&hud_image);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const int margin = 12;
  const QRectF frame_rect(margin, margin, hud_width_ - 2 * margin, hud_height_ - 2 * margin);

  QColor frame_color(80, 200, 255);
  frame_color.setAlpha(static_cast<int>(hud_alpha_ * 255));
  painter.setPen(QPen(frame_color, 2));
  painter.setBrush(Qt::NoBrush);
  painter.drawRoundedRect(frame_rect, 14, 14);

  QFont base_font = painter.font();

  // Title (optional)
  int bar_top = margin + 40;
  if (!title_text_.isEmpty()) {
    QFont title_font = base_font;
    title_font.setPointSize(14);
    title_font.setBold(true);
    painter.setFont(title_font);
    painter.setPen(frame_color);
    painter.drawText(QRectF(margin, margin, hud_width_ - 2 * margin, 26),
                     Qt::AlignHCenter | Qt::AlignVCenter, title_text_);
    bar_top = margin + 34;
    painter.setFont(base_font);
  }

  // Bar geometry
  const int bar_width = std::max(36, hud_width_ / 6);
  const int bar_bottom_margin = 48;
  const int bar_height = std::max(60, hud_height_ - bar_top - margin - bar_bottom_margin);
  const int bar_x = hud_width_ / 2 - bar_width / 2;
  const int bar_y = bar_top + 6;

  const QRect bar_outline(bar_x, bar_y, bar_width, bar_height);
  QColor bar_outline_color = frame_color;
  bar_outline_color.setAlpha(static_cast<int>(hud_alpha_ * 200));
  painter.setPen(QPen(bar_outline_color, 2));
  painter.setBrush(QColor(255, 255, 255, 25));
  painter.drawRoundedRect(bar_outline, 8, 8);

  const double certainty = have_certainty_ ? std::clamp(smoothed_certainty_, 0.0, 1.0) : 0.0;
  const int filled_height = static_cast<int>(certainty * static_cast<double>(bar_height));
  const int fill_top = bar_y + bar_height - filled_height;
  const QRect filled_rect(bar_x + 3, fill_top + 3, bar_width - 6, filled_height - 6);

  if (filled_rect.height() > 0) {
    const bool blink_on = blink_state_ && certainty <= static_cast<double>(blink_threshold_);
    QColor bar_color = barColorForCertainty(certainty, blink_on);
    bar_color.setAlpha(static_cast<int>(hud_alpha_ * 255));

    QLinearGradient gradient(filled_rect.left(), filled_rect.bottom(), filled_rect.right(), filled_rect.top());
    gradient.setColorAt(0.0, bar_color.darker(120));
    gradient.setColorAt(1.0, bar_color.lighter(130));

    painter.setBrush(gradient);
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(filled_rect, 6, 6);
  }

  // Threshold ticks
  painter.setPen(QPen(QColor(255, 255, 255, 120), 1, Qt::DashLine));
  const auto threshold_to_y = [&](float threshold) {
    return bar_y + bar_height - static_cast<int>(threshold * bar_height);
  };

  const int high_y = threshold_to_y(high_threshold_);
  painter.drawLine(bar_x, high_y, bar_x + bar_width, high_y);

  const int mid_y = threshold_to_y(low_threshold_);
  painter.drawLine(bar_x, mid_y, bar_x + bar_width, mid_y);

  // Text readouts
  QFont value_font = painter.font();
  value_font.setPointSize(18);
  value_font.setBold(true);
  painter.setFont(value_font);
  QColor value_color = frame_color;
  value_color.setAlpha(255);
  painter.setPen(value_color);

  const double percentage = certainty * 100.0;
  QString percentage_text = have_certainty_
    ? QString::number(percentage, 'f', 1) + QLatin1String(" %")
    : QStringLiteral("-- %");

  painter.drawText(QRectF(0, bar_y + bar_height + 8, hud_width_, 32),
                   Qt::AlignHCenter | Qt::AlignVCenter, percentage_text);

  painter.end();
  pixel_buffer->unlock();
}

QColor AttentionUncertaintyDisplay::barColorForCertainty(double certainty, bool blink_on) const
{
  QColor color;

  if (certainty >= static_cast<double>(high_threshold_)) {
    color = high_color_;
  } else if (certainty >= static_cast<double>(low_threshold_)) {
    color = mid_color_;
  } else {
    color = low_color_;
  }

  if (blink_on) {
    return color.lighter(blink_state_ ? 180 : 60);
  }

  return color;
}

void AttentionUncertaintyDisplay::updatePosition()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateSize()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateTransparency()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateThresholds()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateSmoothing()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  smoothing_alpha_ = std::clamp(smoothing_alpha_property_->getFloat(), 0.0f, 0.95f);
  update_required_ = true;
}

void AttentionUncertaintyDisplay::updateTitle()
{
  std::lock_guard<std::mutex> lock(hud_mutex_);
  title_text_ = QString::fromStdString(title_text_property_->getStdString());
  update_required_ = true;
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::AttentionUncertaintyDisplay, rviz_common::Display)
