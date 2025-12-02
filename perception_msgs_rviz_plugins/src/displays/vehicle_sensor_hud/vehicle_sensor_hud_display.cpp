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

#include "perception_msgs/displays/vehicle_sensor_hud/vehicle_sensor_hud_display.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <QFont>
#include <QImage>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <OgreTextureManager.h>

#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_system.hpp>
#include <rviz_common/logging.hpp>

namespace perception_msgs {
namespace displays {

static int vehicle_sensor_hud_counter = 0;

VehicleSensorHudDisplay::VehicleSensorHudDisplay()
  : width_(240), height_(280), left_(40), top_(280),
    alpha_(0.9f), bg_alpha_(0.5f), title_("System Health"),
    healthy_color_(0, 220, 120), warning_color_(255, 200, 40),
    unhealthy_color_(255, 70, 70), vehicle_color_(60, 140, 200),
    high_threshold_(0.75f), low_threshold_(0.45f), max_variance_(10.0f),
    lidar_timeout_(1.0f),
    object_count_(0), smoothed_classification_(0.0), smoothed_regression_(0.0),
    has_objectlist_data_(false), objectlist_last_update_(0.0),
    flow_phase_(0.0), pulse_phase_(0.0),
    overlay_(nullptr), panel_(nullptr),
    update_required_(false)
{
  lidar_status_["FR"] = LidarStatus();
  lidar_status_["FL"] = LidarStatus();
  lidar_status_["RR"] = LidarStatus();
  lidar_status_["RL"] = LidarStatus();
  
  const int id = vehicle_sensor_hud_counter++;
  overlay_name_ = "VehicleSensorHudOverlay" + std::to_string(id);
  panel_name_ = "VehicleSensorHudPanel" + std::to_string(id);
  material_name_ = "VehicleSensorHudMaterial" + std::to_string(id);
  texture_name_ = "VehicleSensorHudTexture" + std::to_string(id);

  left_property_ = new rviz_common::properties::IntProperty(
    "Left", left_, "X position", this, SLOT(updatePosition()));
  top_property_ = new rviz_common::properties::IntProperty(
    "Top", top_, "Y position", this, SLOT(updatePosition()));
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", width_, "Width in pixels", this, SLOT(updateSize()));
  height_property_ = new rviz_common::properties::IntProperty(
    "Height", height_, "Height in pixels", this, SLOT(updateSize()));

  alpha_property_ = new rviz_common::properties::FloatProperty(
    "Foreground Alpha", alpha_, "Foreground opacity", this, SLOT(updateAppearance()));
  alpha_property_->setMin(0.0f);
  alpha_property_->setMax(1.0f);
  
  bg_alpha_property_ = new rviz_common::properties::FloatProperty(
    "Background Alpha", bg_alpha_, "Background opacity", this, SLOT(updateAppearance()));
  bg_alpha_property_->setMin(0.0f);
  bg_alpha_property_->setMax(1.0f);
  
  title_property_ = new rviz_common::properties::StringProperty(
    "Title", title_, "HUD title", this, SLOT(updateAppearance()));

  lidar_fr_topic_property_ = new rviz_common::properties::StringProperty(
    "LiDAR FR Topic", "/drivers/ouster_lidar_fr/points", "Front-right LiDAR", 
    this, SLOT(updateLidarTopics()));
  lidar_fl_topic_property_ = new rviz_common::properties::StringProperty(
    "LiDAR FL Topic", "/drivers/ouster_lidar_fl/points", "Front-left LiDAR",
    this, SLOT(updateLidarTopics()));
  lidar_rr_topic_property_ = new rviz_common::properties::StringProperty(
    "LiDAR RR Topic", "/drivers/ouster_lidar_rr/points", "Rear-right LiDAR",
    this, SLOT(updateLidarTopics()));
  lidar_rl_topic_property_ = new rviz_common::properties::StringProperty(
    "LiDAR RL Topic", "/drivers/ouster_lidar_rl/points", "Rear-left LiDAR",
    this, SLOT(updateLidarTopics()));

  healthy_color_property_ = new rviz_common::properties::ColorProperty(
    "Healthy Color", healthy_color_, "Color for healthy status", this, SLOT(updateAppearance()));
  warning_color_property_ = new rviz_common::properties::ColorProperty(
    "Warning Color", warning_color_, "Color for warning status", this, SLOT(updateAppearance()));
  unhealthy_color_property_ = new rviz_common::properties::ColorProperty(
    "Unhealthy Color", unhealthy_color_, "Color for unhealthy status", this, SLOT(updateAppearance()));
  vehicle_color_property_ = new rviz_common::properties::ColorProperty(
    "Vehicle Color", vehicle_color_, "Wireframe color", this, SLOT(updateAppearance()));

  high_threshold_property_ = new rviz_common::properties::FloatProperty(
    "High Threshold", high_threshold_, "Threshold for healthy (0-1)", this, SLOT(updateAppearance()));
  low_threshold_property_ = new rviz_common::properties::FloatProperty(
    "Low Threshold", low_threshold_, "Threshold for warning (0-1)", this, SLOT(updateAppearance()));
  max_variance_property_ = new rviz_common::properties::FloatProperty(
    "Max Variance", max_variance_, "Max variance for normalization", this, SLOT(updateAppearance()));
  lidar_timeout_property_ = new rviz_common::properties::FloatProperty(
    "LiDAR Timeout", lidar_timeout_, "Seconds before LiDAR inactive", this, SLOT(updateAppearance()));
}

VehicleSensorHudDisplay::~VehicleSensorHudDisplay() {
  destroyLidarSubscriptions();
  destroyOverlay();
  
  delete width_property_;
  delete height_property_;
  delete left_property_;
  delete top_property_;
  delete alpha_property_;
  delete bg_alpha_property_;
  delete title_property_;
  delete lidar_fr_topic_property_;
  delete lidar_fl_topic_property_;
  delete lidar_rr_topic_property_;
  delete lidar_rl_topic_property_;
  delete healthy_color_property_;
  delete warning_color_property_;
  delete unhealthy_color_property_;
  delete vehicle_color_property_;
  delete high_threshold_property_;
  delete low_threshold_property_;
  delete max_variance_property_;
  delete lidar_timeout_property_;
}

void VehicleSensorHudDisplay::onInitialize() {
  RVIZ_COMMON_LOG_INFO("VehicleSensorHudDisplay::onInitialize() called");
  MessageFilterDisplay::onInitialize();
  rviz_rendering::RenderSystem::get()->prepareOverlays(context_->getSceneManager());
  createOverlay();
  createLidarSubscriptions();
  RVIZ_COMMON_LOG_INFO("VehicleSensorHudDisplay::onInitialize() complete");
}

void VehicleSensorHudDisplay::onEnable() {
  RVIZ_COMMON_LOG_INFO("VehicleSensorHudDisplay::onEnable() called");
  MessageFilterDisplay::onEnable();
  if (overlay_) overlay_->show();
  createLidarSubscriptions();
  update_required_ = true;
}

void VehicleSensorHudDisplay::onDisable() {
  MessageFilterDisplay::onDisable();
  if (overlay_) overlay_->hide();
  destroyLidarSubscriptions();
}

void VehicleSensorHudDisplay::createLidarSubscriptions() {
  auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  auto qos = rclcpp::SensorDataQoS();
  
  auto create_sub = [&](const std::string& topic, const std::string& id) {
    if (!topic.empty()) {
      return node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos, [this, id](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          lidarCallback(id, msg);
        });
    }
    return rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr();
  };
  
  lidar_fr_sub_ = create_sub(lidar_fr_topic_property_->getStdString(), "FR");
  lidar_fl_sub_ = create_sub(lidar_fl_topic_property_->getStdString(), "FL");
  lidar_rr_sub_ = create_sub(lidar_rr_topic_property_->getStdString(), "RR");
  lidar_rl_sub_ = create_sub(lidar_rl_topic_property_->getStdString(), "RL");
}

void VehicleSensorHudDisplay::destroyLidarSubscriptions() {
  lidar_fr_sub_.reset();
  lidar_fl_sub_.reset();
  lidar_rr_sub_.reset();
  lidar_rl_sub_.reset();
}

void VehicleSensorHudDisplay::lidarCallback(const std::string& sensor_id,
    sensor_msgs::msg::PointCloud2::ConstSharedPtr /*msg*/) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  lidar_status_[sensor_id].active = true;
  lidar_status_[sensor_id].last_msg_time = 0.0;
  update_required_ = true;
}

void VehicleSensorHudDisplay::updateLidarTopics() {
  destroyLidarSubscriptions();
  createLidarSubscriptions();
}

static int update_debug_counter = 0;

void VehicleSensorHudDisplay::update(float wall_dt, float /*ros_dt*/) {
  // Animation phases
  flow_phase_ += wall_dt * 1.5;
  if (flow_phase_ > 1.0) flow_phase_ -= 1.0;
  
  pulse_phase_ += wall_dt * 3.0;
  if (pulse_phase_ > 2.0 * M_PI) pulse_phase_ -= 2.0 * M_PI;
  
  // Update timeouts
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& pair : lidar_status_) {
      pair.second.last_msg_time += wall_dt;
      if (pair.second.last_msg_time > lidar_timeout_) {
        pair.second.active = false;
      }
    }
    objectlist_last_update_ += wall_dt;
    if (objectlist_last_update_ > 2.0) {
      has_objectlist_data_ = false;
    }
    
    // Debug print every ~60 frames
    if (++update_debug_counter % 60 == 0) {
      RVIZ_COMMON_LOG_INFO_STREAM("VehicleSensorHudDisplay::update() has_objectlist_data_=" 
                                   << has_objectlist_data_ 
                                   << " objectlist_last_update_=" << objectlist_last_update_);
    }
  }
  
  update_required_ = true;
  if (update_required_ && isEnabled()) {
    updateHud();
    update_required_ = false;
  }
}

void VehicleSensorHudDisplay::createOverlay() {
  Ogre::OverlayManager& mgr = *Ogre::OverlayManager::getSingletonPtr();
  
  overlay_ = mgr.create(overlay_name_);
  panel_ = static_cast<Ogre::PanelOverlayElement*>(
    mgr.createOverlayElement("Panel", panel_name_));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setPosition(left_, top_);
  panel_->setDimensions(width_, height_);
  
  material_ = Ogre::MaterialManager::getSingleton().create(
    material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material_->setReceiveShadows(false);
  material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material_->setCullingMode(Ogre::CULL_NONE);
  
  texture_ = Ogre::TextureManager::getSingleton().createManual(
    texture_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DYNAMIC_WRITE_ONLY);
  
  material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
  panel_->setMaterialName(material_name_);
  
  overlay_->add2D(panel_);
  overlay_->setZOrder(500);
  overlay_->show();
}

void VehicleSensorHudDisplay::destroyOverlay() {
  if (overlay_) {
    Ogre::OverlayManager& mgr = *Ogre::OverlayManager::getSingletonPtr();
    if (panel_) {
      overlay_->remove2D(panel_);
      mgr.destroyOverlayElement(panel_);
      panel_ = nullptr;
    }
    mgr.destroy(overlay_);
    overlay_ = nullptr;
  }
  if (material_) {
    Ogre::MaterialManager::getSingleton().remove(material_name_);
    material_.reset();
  }
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_name_);
    texture_.reset();
  }
}

void VehicleSensorHudDisplay::updatePosition() {
  left_ = left_property_->getInt();
  top_ = top_property_->getInt();
  if (panel_) panel_->setPosition(left_, top_);
  update_required_ = true;
}

void VehicleSensorHudDisplay::updateSize() {
  int new_w = std::max(150, width_property_->getInt());
  int new_h = std::max(150, height_property_->getInt());
  
  if (new_w != width_ || new_h != height_) {
    width_ = new_w;
    height_ = new_h;
    
    if (texture_) {
      material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
      Ogre::TextureManager::getSingleton().remove(texture_name_);
      texture_ = Ogre::TextureManager::getSingleton().createManual(
        texture_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width_, height_, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DYNAMIC_WRITE_ONLY);
      material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name_);
    }
    if (panel_) panel_->setDimensions(width_, height_);
  }
  update_required_ = true;
}

void VehicleSensorHudDisplay::updateAppearance() {
  alpha_ = alpha_property_->getFloat();
  bg_alpha_ = bg_alpha_property_->getFloat();
  title_ = title_property_->getString();
  healthy_color_ = healthy_color_property_->getColor();
  warning_color_ = warning_color_property_->getColor();
  unhealthy_color_ = unhealthy_color_property_->getColor();
  vehicle_color_ = vehicle_color_property_->getColor();
  high_threshold_ = high_threshold_property_->getFloat();
  low_threshold_ = low_threshold_property_->getFloat();
  max_variance_ = std::max(0.1f, max_variance_property_->getFloat());
  lidar_timeout_ = std::max(0.1f, lidar_timeout_property_->getFloat());
  update_required_ = true;
}

void VehicleSensorHudDisplay::processMessage(
    perception_msgs::msg::ObjectList::ConstSharedPtr msg) {
  RVIZ_COMMON_LOG_INFO_STREAM("VehicleSensorHudDisplay::processMessage() this=" << this
                               << " received msg with " << msg->objects.size() << " objects");
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  object_count_ = static_cast<int>(msg->objects.size());
  
  double total_class = 0.0, total_regr = 0.0;
  int valid_class = 0, valid_regr = 0;
  
  for (const auto& obj : msg->objects) {
    // Classification
    double max_prob = 0.0;
    for (const auto& c : obj.state.classifications) {
      if (std::isfinite(c.probability)) {
        max_prob = std::max(max_prob, static_cast<double>(c.probability));
      }
    }
    if (max_prob > 0.0) {
      total_class += std::clamp(max_prob, 0.0, 1.0);
      ++valid_class;
    }
    
    // Regression
    const auto& cov = obj.state.continuous_state_covariance;
    const int ss = static_cast<int>(obj.state.continuous_state.size());
    if (cov.size() >= static_cast<size_t>(ss * ss) && ss >= 12) {
      const std::array<int, 7> idx = {0, 1, 2, 7, 9, 10, 11};
      double sum_var = 0.0;
      int cnt = 0;
      for (int i : idx) {
        double v = cov[i * ss + i];
        if (std::isfinite(v) && v >= 0.0) { sum_var += v; ++cnt; }
      }
      if (cnt > 0) {
        double cert = std::exp(-(sum_var / cnt) / max_variance_);
        total_regr += std::clamp(cert, 0.0, 1.0);
        ++valid_regr;
      }
    }
  }
  
  double class_cert = valid_class > 0 ? total_class / valid_class : 0.0;
  double regr_cert = valid_regr > 0 ? total_regr / valid_regr : 0.0;
  
  const double s = 0.8;
  smoothed_classification_ = s * smoothed_classification_ + (1.0 - s) * class_cert;
  smoothed_regression_ = s * smoothed_regression_ + (1.0 - s) * regr_cert;
  
  has_objectlist_data_ = true;
  objectlist_last_update_ = 0.0;
  update_required_ = true;
  
  RVIZ_COMMON_LOG_INFO_STREAM("VehicleSensorHudDisplay: has_objectlist_data_=" << has_objectlist_data_ 
                               << " class=" << smoothed_classification_ 
                               << " regr=" << smoothed_regression_);
}

double VehicleSensorHudDisplay::computeOverallCertainty() const {
  return (smoothed_classification_ + smoothed_regression_) / 2.0;
}

void VehicleSensorHudDisplay::updateHud() {
  if (!texture_) return;
  
  Ogre::HardwarePixelBufferSharedPtr buffer = texture_->getBuffer();
  buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& pb = buffer->getCurrentLock();
  
  QImage image(static_cast<uchar*>(pb.data), width_, height_, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);
  drawHud(painter);
  painter.end();
  
  buffer->unlock();
}

void VehicleSensorHudDisplay::drawHud(QPainter& painter) {
  const int margin = 8;
  
  // Read data under lock first
  bool has_data;
  double cls_val, reg_val;
  int obj_count;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    has_data = has_objectlist_data_;
    cls_val = smoothed_classification_;
    reg_val = smoothed_regression_;
    obj_count = object_count_;
  }
  
  // Debug - log this pointer and data
  static int draw_counter = 0;
  if (++draw_counter % 60 == 0) {
    RVIZ_COMMON_LOG_INFO_STREAM("VehicleSensorHudDisplay::drawHud() this=" << this 
                                 << " has_data=" << has_data << " cls=" << cls_val);
  }
  
  // Background
  QLinearGradient bg(0, 0, 0, height_);
  bg.setColorAt(0.0, QColor(15, 25, 40, static_cast<int>(bg_alpha_ * 255 * 0.95)));
  bg.setColorAt(0.5, QColor(20, 35, 55, static_cast<int>(bg_alpha_ * 255)));
  bg.setColorAt(1.0, QColor(10, 20, 35, static_cast<int>(bg_alpha_ * 255 * 0.9)));
  
  QPainterPath bg_path;
  bg_path.addRoundedRect(QRectF(0, 0, width_, height_), 8, 8);
  painter.fillPath(bg_path, bg);
  
  // Borders
  painter.setPen(QPen(QColor(60, 140, 200, static_cast<int>(alpha_ * 80)), 1));
  painter.drawPath(bg_path);
  
  // Title - show live data status
  QFont title_font("Segoe UI", 9, QFont::Bold);
  title_font.setLetterSpacing(QFont::PercentageSpacing, 105);
  painter.setFont(title_font);
  
  QString title_text;
  if (has_data) {
    title_text = QString("OBJ:%1 CLS:%2% REG:%3%")
      .arg(obj_count)
      .arg(static_cast<int>(cls_val * 100))
      .arg(static_cast<int>(reg_val * 100));
    painter.setPen(QColor(0, 220, 120, static_cast<int>(alpha_ * 255)));
  } else {
    title_text = "NO DATA";
    painter.setPen(QColor(255, 70, 70, static_cast<int>(alpha_ * 255)));
  }
  painter.drawText(QRectF(margin, margin, width_ - 2*margin, 20),
                   Qt::AlignHCenter | Qt::AlignVCenter, title_text);
  
  // Vehicle area
  const int title_h = 28;
  const int status_h = 70;
  QRectF veh_bounds(margin, title_h, width_ - 2*margin, height_ - title_h - status_h - margin);
  
  drawWireframeVan(painter, veh_bounds);
  
  // Status panel
  QRectF status_bounds(margin, height_ - status_h - margin, width_ - 2*margin, status_h);
  drawStatusPanel(painter, status_bounds);
}

void VehicleSensorHudDisplay::drawWireframeVan(QPainter& painter, const QRectF& bounds) {
  const double cx = bounds.center().x();
  const double cy = bounds.center().y();
  
  // Isometric projection factors
  const double iso_x = 0.866;  // cos(30°)
  const double iso_y = 0.5;    // sin(30°)
  
  // Van dimensions in isometric space
  const double length = bounds.width() * 0.6;
  const double width = bounds.width() * 0.35;
  const double height = bounds.height() * 0.25;
  
  // Helper to convert 3D to 2D isometric
  auto to2D = [&](double x, double y, double z) -> QPointF {
    double px = cx + (x - y) * iso_x;
    double py = cy + (x + y) * iso_y - z;
    return QPointF(px, py);
  };
  
  // Van vertices (centered at origin, front facing left-down)
  const double l2 = length / 2;
  const double w2 = width / 2;
  const double h = height;
  const double roof_h = h * 0.7;
  
  // Bottom corners (z=0)
  QPointF b_fl = to2D(-l2, -w2, 0);  // front-left
  QPointF b_fr = to2D(-l2, w2, 0);   // front-right
  QPointF b_rl = to2D(l2, -w2, 0);   // rear-left
  QPointF b_rr = to2D(l2, w2, 0);    // rear-right
  
  // Top corners (z=height)
  QPointF t_fl = to2D(-l2, -w2, h);
  QPointF t_fr = to2D(-l2, w2, h);
  QPointF t_rl = to2D(l2, -w2, h);
  QPointF t_rr = to2D(l2, w2, h);
  
  // Roof corners (lower than top, for van shape)
  QPointF r_fl = to2D(-l2 * 0.6, -w2, h + roof_h);
  QPointF r_fr = to2D(-l2 * 0.6, w2, h + roof_h);
  QPointF r_rl = to2D(l2, -w2, h + roof_h);
  QPointF r_rr = to2D(l2, w2, h + roof_h);
  
  // Wireframe style
  QColor wire_color = vehicle_color_;
  wire_color.setAlpha(static_cast<int>(alpha_ * 200));
  QPen wire_pen(wire_color, 1.5);
  painter.setPen(wire_pen);
  painter.setBrush(Qt::NoBrush);
  
  // Draw bottom rectangle
  painter.drawLine(b_fl, b_fr);
  painter.drawLine(b_fr, b_rr);
  painter.drawLine(b_rr, b_rl);
  painter.drawLine(b_rl, b_fl);
  
  // Draw vertical edges
  painter.drawLine(b_fl, t_fl);
  painter.drawLine(b_fr, t_fr);
  painter.drawLine(b_rl, t_rl);
  painter.drawLine(b_rr, t_rr);
  
  // Draw middle rectangle (body top)
  painter.drawLine(t_fl, t_fr);
  painter.drawLine(t_fr, t_rr);
  painter.drawLine(t_rr, t_rl);
  painter.drawLine(t_rl, t_fl);
  
  // Draw roof edges
  painter.drawLine(t_fl, r_fl);
  painter.drawLine(t_fr, r_fr);
  painter.drawLine(t_rl, r_rl);
  painter.drawLine(t_rr, r_rr);
  
  // Draw roof rectangle
  painter.drawLine(r_fl, r_fr);
  painter.drawLine(r_fr, r_rr);
  painter.drawLine(r_rr, r_rl);
  painter.drawLine(r_rl, r_fl);
  
  // Windshield line
  QColor accent(100, 180, 255, static_cast<int>(alpha_ * 150));
  painter.setPen(QPen(accent, 1));
  painter.drawLine(r_fl, r_fr);
  
  // Center point for data flow
  QPointF center = to2D(0, 0, h * 0.5);
  
  // LiDAR positions (at corners)
  QPointF lidar_fl = to2D(-l2 * 0.9, -w2 * 0.9, h * 0.3);
  QPointF lidar_fr = to2D(-l2 * 0.9, w2 * 0.9, h * 0.3);
  QPointF lidar_rl = to2D(l2 * 0.9, -w2 * 0.9, h * 0.3);
  QPointF lidar_rr = to2D(l2 * 0.9, w2 * 0.9, h * 0.3);
  
  // Get LiDAR status and objectlist data under lock
  bool fl_active, fr_active, rl_active, rr_active;
  bool has_data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    fl_active = lidar_status_["FL"].active;
    fr_active = lidar_status_["FR"].active;
    rl_active = lidar_status_["RL"].active;
    rr_active = lidar_status_["RR"].active;
    has_data = has_objectlist_data_;
  }
  
  // Draw data flow lines (animated)
  drawDataFlowLine(painter, lidar_fl, center, fl_active, flow_phase_);
  drawDataFlowLine(painter, lidar_fr, center, fr_active, flow_phase_ + 0.25);
  drawDataFlowLine(painter, lidar_rl, center, rl_active, flow_phase_ + 0.5);
  drawDataFlowLine(painter, lidar_rr, center, rr_active, flow_phase_ + 0.75);
  
  // Draw LiDAR indicators
  drawHealthIndicator(painter, lidar_fl, "FL", fl_active);
  drawHealthIndicator(painter, lidar_fr, "FR", fr_active);
  drawHealthIndicator(painter, lidar_rl, "RL", rl_active);
  drawHealthIndicator(painter, lidar_rr, "RR", rr_active);
  
  // Center processing indicator
  double pulse = 0.5 + 0.5 * std::sin(pulse_phase_);
  int active_count = (fl_active ? 1 : 0) + (fr_active ? 1 : 0) + 
                     (rl_active ? 1 : 0) + (rr_active ? 1 : 0);
  
  QColor center_color;
  if (active_count == 4 && has_data) {
    center_color = healthy_color_;
  } else if (active_count >= 2) {
    center_color = warning_color_;
  } else {
    center_color = unhealthy_color_;
  }
  
  // Glow
  QRadialGradient glow(center, 20);
  QColor glow_col = center_color;
  glow_col.setAlpha(static_cast<int>(alpha_ * 60 * pulse));
  glow.setColorAt(0.0, glow_col);
  glow.setColorAt(1.0, Qt::transparent);
  painter.setBrush(glow);
  painter.setPen(Qt::NoPen);
  painter.drawEllipse(center, 20, 20);
  
  // Core
  QRadialGradient core(center, 8);
  core.setColorAt(0.0, center_color.lighter(150));
  core.setColorAt(0.7, center_color);
  core.setColorAt(1.0, center_color.darker(120));
  painter.setBrush(core);
  painter.setPen(QPen(center_color.lighter(130), 1));
  painter.drawEllipse(center, 8, 8);
  
  // Label
  QFont font("Consolas", 6, QFont::Bold);
  painter.setFont(font);
  painter.setPen(QColor(180, 210, 240, static_cast<int>(alpha_ * 200)));
  painter.drawText(QRectF(center.x() - 15, center.y() + 12, 30, 12), 
                   Qt::AlignCenter, "PROC");
}

void VehicleSensorHudDisplay::drawDataFlowLine(QPainter& painter, 
    const QPointF& from, const QPointF& to, bool healthy, double phase) {
  
  QColor line_color = healthy ? healthy_color_ : unhealthy_color_;
  
  if (healthy) {
    // Animated flowing line - draw multiple segments
    const int segments = 8;
    const double seg_len = 1.0 / segments;
    
    for (int i = 0; i < segments; ++i) {
      double t_start = std::fmod(phase + i * seg_len, 1.0);
      double t_end = std::fmod(phase + (i + 0.5) * seg_len, 1.0);
      
      // Only draw if segment is moving towards center
      if (t_start < t_end) {
        QPointF p1(from.x() + (to.x() - from.x()) * t_start,
                   from.y() + (to.y() - from.y()) * t_start);
        QPointF p2(from.x() + (to.x() - from.x()) * t_end,
                   from.y() + (to.y() - from.y()) * t_end);
        
        // Fade based on position
        int alpha_val = static_cast<int>(alpha_ * 180 * (0.3 + 0.7 * t_end));
        QColor seg_color = line_color;
        seg_color.setAlpha(alpha_val);
        
        painter.setPen(QPen(seg_color, 2, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(p1, p2);
      }
    }
    
    // Subtle base line
    QColor base_color = line_color;
    base_color.setAlpha(static_cast<int>(alpha_ * 30));
    painter.setPen(QPen(base_color, 1));
    painter.drawLine(from, to);
    
  } else {
    // Broken/dashed red line
    QColor broken_color = line_color;
    broken_color.setAlpha(static_cast<int>(alpha_ * 150));
    
    QPen broken_pen(broken_color, 1.5, Qt::DashLine);
    broken_pen.setDashPattern({3, 4});
    painter.setPen(broken_pen);
    painter.drawLine(from, to);
    
    // X mark at midpoint
    QPointF mid((from.x() + to.x()) / 2, (from.y() + to.y()) / 2);
    painter.setPen(QPen(unhealthy_color_, 2));
    painter.drawLine(mid.x() - 4, mid.y() - 4, mid.x() + 4, mid.y() + 4);
    painter.drawLine(mid.x() + 4, mid.y() - 4, mid.x() - 4, mid.y() + 4);
  }
}

void VehicleSensorHudDisplay::drawHealthIndicator(QPainter& painter, 
    const QPointF& pos, const QString& label, bool active) {
  
  QColor color = active ? healthy_color_ : unhealthy_color_;
  const double size = 5;
  
  // Small indicator dot
  QRadialGradient grad(pos, size);
  grad.setColorAt(0.0, active ? color.lighter(140) : QColor(60, 60, 60));
  grad.setColorAt(0.7, active ? color : QColor(40, 40, 40));
  grad.setColorAt(1.0, active ? color.darker(120) : QColor(30, 30, 30));
  
  painter.setBrush(grad);
  painter.setPen(QPen(color.lighter(130), 1));
  painter.drawEllipse(pos, size, size);
  
  // Label
  QFont font("Consolas", 5);
  painter.setFont(font);
  QColor label_color = active ? color.lighter(120) : QColor(100, 100, 100);
  label_color.setAlpha(static_cast<int>(alpha_ * 200));
  painter.setPen(label_color);
  painter.drawText(QRectF(pos.x() - 8, pos.y() + size + 1, 16, 8), 
                   Qt::AlignCenter, label);
}

void VehicleSensorHudDisplay::drawStatusPanel(QPainter& painter, const QRectF& bounds) {
  // Background
  QPainterPath panel;
  panel.addRoundedRect(bounds, 6, 6);
  painter.fillPath(panel, QColor(10, 20, 35, static_cast<int>(alpha_ * 150)));
  painter.setPen(QPen(QColor(50, 100, 150, static_cast<int>(alpha_ * 80)), 1));
  painter.drawPath(panel);
  
  const double row_h = bounds.height() / 4;
  
  QFont label_font("Segoe UI", 7);
  QFont value_font("Consolas", 8, QFont::Bold);
  
  auto drawRow = [&](int row, const QString& lbl, const QString& val, const QColor& col) {
    double y = bounds.top() + row * row_h;
    painter.setFont(label_font);
    painter.setPen(QColor(140, 170, 200, static_cast<int>(alpha_ * 200)));
    painter.drawText(QRectF(bounds.left() + 6, y, bounds.width() * 0.5, row_h),
                     Qt::AlignLeft | Qt::AlignVCenter, lbl);
    painter.setFont(value_font);
    QColor c = col; c.setAlpha(static_cast<int>(alpha_ * 255));
    painter.setPen(c);
    painter.drawText(QRectF(bounds.left() + bounds.width() * 0.5, y, 
                            bounds.width() * 0.5 - 6, row_h),
                     Qt::AlignRight | Qt::AlignVCenter, val);
  };
  
  // Read all shared data under lock
  int active = 0;
  bool has_data = false;
  double cls_val = 0.0;
  double reg_val = 0.0;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& p : lidar_status_) {
      if (p.second.active) ++active;
    }
    has_data = has_objectlist_data_;
    cls_val = smoothed_classification_;
    reg_val = smoothed_regression_;
  }
  
  // Status
  QColor status_col;
  QString status_txt;
  if (!has_data) {
    status_col = unhealthy_color_;
    status_txt = "NO DATA";
  } else {
    double overall = (cls_val + reg_val) / 2.0;
    if (overall >= high_threshold_ && active == 4) {
      status_col = healthy_color_;
      status_txt = "HEALTHY";
    } else if (overall >= low_threshold_ && active >= 2) {
      status_col = warning_color_;
      status_txt = "DEGRADED";
    } else {
      status_col = unhealthy_color_;
      status_txt = "CRITICAL";
    }
  }
  
  QColor lidar_col = (active == 4) ? healthy_color_ : 
                     (active >= 2) ? warning_color_ : unhealthy_color_;
  
  drawRow(0, "LIDAR:", QString("%1/4 ACTIVE").arg(active), lidar_col);
  drawRow(1, "STATUS:", status_txt, status_col);
  drawRow(2, "CLS:", has_data ? 
          QString::number(static_cast<int>(cls_val * 100)) + "%" : "--%",
          QColor(180, 210, 240));
  drawRow(3, "REG:", has_data ?
          QString::number(static_cast<int>(reg_val * 100)) + "%" : "--%",
          QColor(180, 120, 255));
}

}  // namespace displays
}  // namespace perception_msgs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(perception_msgs::displays::VehicleSensorHudDisplay, rviz_common::Display)
