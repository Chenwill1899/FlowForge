#include "world_joystick_display.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <sstream>

#include <QFont>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPolygonF>
#include <QRadialGradient>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgrePixelFormat.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>
#include <OgrePrerequisites.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <pluginlib/class_list_macros.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>

namespace
{
constexpr int kTextureSize = 240;
constexpr int kMarginPixels = 18;

double clamp(double value, double minimum, double maximum)
{
  return std::max(minimum, std::min(maximum, value));
}

QString signedValue(double value)
{
  return QString::number(value, 'f', 2).prepend(value >= 0.0 ? "+" : "");
}
}  // namespace

namespace rviz
{

WorldJoystickDisplay::WorldJoystickDisplay()
  : topic_property_(new RosTopicProperty(
        "Topic", "/human_intent", "geometry_msgs/TwistStamped",
        "Final planar velocity intent.",
        this, SLOT(updateTopic())))
  , expected_frame_property_(new StringProperty(
        "Expected Frame", "world",
        "Frame_id required on the velocity intent.", this))
  , full_scale_speed_property_(new FloatProperty(
        "Full Scale Speed", 1.0,
        "Velocity magnitude represented by the outer joystick ring (m/s).",
        this, SLOT(updateScale())))
  , have_intent_(false)
  , frame_valid_(true)
  , dirty_(true)
  , overlay_(nullptr)
  , panel_(nullptr)
{
  full_scale_speed_property_->setMin(0.01);

  std::ostringstream suffix;
  suffix << reinterpret_cast<std::uintptr_t>(this);
  overlay_name_ = "WorldJoystickOverlay/" + suffix.str();
  panel_name_ = "WorldJoystickPanel/" + suffix.str();
  texture_name_ = "WorldJoystickTexture/" + suffix.str();
  material_name_ = "WorldJoystickMaterial/" + suffix.str();
}

WorldJoystickDisplay::~WorldJoystickDisplay()
{
  unsubscribe();
  destroyOverlay();
}

void WorldJoystickDisplay::onInitialize()
{
  createOverlay();
  redraw();
}

void WorldJoystickDisplay::onEnable()
{
  if (overlay_)
  {
    overlay_->show();
  }
  subscribe();
  dirty_ = true;
}

void WorldJoystickDisplay::onDisable()
{
  unsubscribe();
  if (overlay_)
  {
    overlay_->hide();
  }
}

void WorldJoystickDisplay::reset()
{
  Display::reset();
  latest_intent_ = geometry_msgs::TwistStamped();
  have_intent_ = false;
  frame_valid_ = true;
  dirty_ = true;
  setStatus(StatusProperty::Warn, "Input", "Waiting for /human_intent");
}

void WorldJoystickDisplay::update(float, float)
{
  if (dirty_)
  {
    redraw();
    dirty_ = false;
  }
}

void WorldJoystickDisplay::updateTopic()
{
  unsubscribe();
  if (isEnabled())
  {
    subscribe();
  }
  have_intent_ = false;
  dirty_ = true;
}

void WorldJoystickDisplay::updateScale()
{
  dirty_ = true;
  queueRender();
}

void WorldJoystickDisplay::subscribe()
{
  const std::string topic = topic_property_->getTopicStd();
  if (topic.empty())
  {
    setStatus(StatusProperty::Warn, "Input", "No topic configured");
    return;
  }

  try
  {
    intent_sub_ = update_nh_.subscribe(
        topic, 1, &WorldJoystickDisplay::incomingIntent, this);
    setStatus(StatusProperty::Warn, "Input", "Waiting for a world-frame intent");
  }
  catch (const ros::Exception& exception)
  {
    setStatus(StatusProperty::Error, "Input",
              QString("Subscription failed: ") + exception.what());
  }
}

void WorldJoystickDisplay::unsubscribe()
{
  intent_sub_.shutdown();
}

void WorldJoystickDisplay::incomingIntent(
    const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  latest_intent_ = *msg;
  have_intent_ = true;
  const std::string expected_frame = expected_frame_property_->getStdString();
  frame_valid_ = msg->header.frame_id == expected_frame ||
                 msg->header.frame_id == "/" + expected_frame;
  dirty_ = true;

  if (frame_valid_)
  {
    setStatus(StatusProperty::Ok, "Input", "Expected-frame intent received");
  }
  else
  {
    setStatus(StatusProperty::Error, "Input",
              QString("Expected frame '%1', received '%2'")
                  .arg(QString::fromStdString(expected_frame))
                  .arg(QString::fromStdString(msg->header.frame_id)));
  }
  queueRender();
}

void WorldJoystickDisplay::createOverlay()
{
  texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, kTextureSize, kTextureSize, 0,
      Ogre::PF_BYTE_BGRA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);

  material_ = Ogre::MaterialManager::getSingleton().create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setDepthCheckEnabled(false);
  pass->setDepthWriteEnabled(false);
  pass->setCullingMode(Ogre::CULL_NONE);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  pass->createTextureUnitState(texture_name_);

  Ogre::OverlayManager& manager = Ogre::OverlayManager::getSingleton();
  panel_ = static_cast<Ogre::PanelOverlayElement*>(
      manager.createOverlayElement("Panel", panel_name_));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);
  panel_->setHorizontalAlignment(Ogre::GHA_LEFT);
  panel_->setVerticalAlignment(Ogre::GVA_BOTTOM);
  panel_->setPosition(kMarginPixels, -(kTextureSize + kMarginPixels));
  panel_->setDimensions(kTextureSize, kTextureSize);
  panel_->setMaterialName(material_name_);

  overlay_ = manager.create(overlay_name_);
  overlay_->setZOrder(500);
  overlay_->add2D(panel_);
}

void WorldJoystickDisplay::destroyOverlay()
{
  Ogre::OverlayManager& manager = Ogre::OverlayManager::getSingleton();

  if (overlay_)
  {
    overlay_->hide();
    if (panel_)
    {
      overlay_->remove2D(panel_);
    }
    manager.destroy(overlay_);
    overlay_ = nullptr;
  }
  if (panel_)
  {
    manager.destroyOverlayElement(panel_);
    panel_ = nullptr;
  }

  if (!material_.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(material_name_);
    material_.setNull();
  }
  if (!texture_.isNull())
  {
    Ogre::TextureManager::getSingleton().remove(texture_name_);
    texture_.setNull();
  }
}

void WorldJoystickDisplay::redraw()
{
  if (texture_.isNull())
  {
    return;
  }

  const double vx = have_intent_ && frame_valid_
                        ? latest_intent_.twist.linear.x : 0.0;
  const double vy = have_intent_ && frame_valid_
                        ? latest_intent_.twist.linear.y : 0.0;
  const double full_scale =
      std::max(0.01, static_cast<double>(full_scale_speed_property_->getFloat()));
  const double speed = std::hypot(vx, vy);
  const double normalized_speed = clamp(speed / full_scale, 0.0, 1.0);
  const double direction_x = speed > 1e-9 ? vx / speed : 0.0;
  const double direction_y = speed > 1e-9 ? vy / speed : 0.0;

  QImage image(kTextureSize, kTextureSize, QImage::Format_ARGB32);
  image.fill(Qt::transparent);

  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing, true);

  constexpr double center_x = 120.0;
  constexpr double center_y = 104.0;
  constexpr double outer_radius = 72.0;
  constexpr double knob_radius = 24.0;
  constexpr double knob_travel = 44.0;

  const QPointF center(center_x, center_y);
  const QPointF knob(
      center_x - direction_y * normalized_speed * knob_travel,
      center_y - direction_x * normalized_speed * knob_travel);

  // ---- Circular frosted-glass pad. A true backdrop blur is impossible from
  // an RViz overlay (no access to the scene pixels behind), so the material
  // reads as glass through the other Apple cues: a bright low-opacity white
  // fill, a hairline border lit from the top, and a wide diffuse shadow.
  const double pad_radius = 86.0;

  painter.setPen(Qt::NoPen);
  // Diffuse drop shadow: stacked translucent discs approximate a blur.
  for (int layer = 3; layer >= 0; --layer)
  {
    const double grow = 1.5 + 2.0 * layer;
    painter.setBrush(QColor(15, 25, 35, 20 - 4 * layer));
    painter.drawEllipse(center + QPointF(0.0, 2.5),
                        pad_radius + grow, pad_radius + grow);
  }

  QLinearGradient pad_fill(center - QPointF(0.0, pad_radius),
                           center + QPointF(0.0, pad_radius));
  pad_fill.setColorAt(0.0, QColor(255, 255, 255, 158));
  pad_fill.setColorAt(0.55, QColor(244, 248, 252, 132));
  pad_fill.setColorAt(1.0, QColor(230, 237, 244, 118));
  painter.setBrush(pad_fill);
  painter.drawEllipse(center, pad_radius, pad_radius);

  // Hairline border, brightest along the top edge (top light source).
  QLinearGradient border_grad(center - QPointF(0.0, pad_radius),
                              center + QPointF(0.0, pad_radius));
  border_grad.setColorAt(0.0, QColor(255, 255, 255, 215));
  border_grad.setColorAt(0.35, QColor(255, 255, 255, 95));
  border_grad.setColorAt(1.0, QColor(255, 255, 255, 45));
  painter.setPen(QPen(QBrush(border_grad), 1.2));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(center, pad_radius - 0.6, pad_radius - 0.6);

  // ---- Recessed dish the knob travels in: darker toward the rim.
  QRadialGradient dish(center, outer_radius);
  dish.setColorAt(0.0, QColor(255, 255, 255, 28));
  dish.setColorAt(0.75, QColor(198, 210, 222, 62));
  dish.setColorAt(1.0, QColor(152, 167, 182, 98));
  painter.setPen(Qt::NoPen);
  painter.setBrush(dish);
  painter.drawEllipse(center, outer_radius, outer_radius);
  painter.setPen(QPen(QColor(122, 138, 154, 70), 1.2));
  painter.setBrush(Qt::NoBrush);
  painter.drawEllipse(center, outer_radius, outer_radius);

  // Faint crosshair and half-scale ring; guides, not chrome.
  painter.setPen(QPen(QColor(96, 112, 128, 42), 1.0));
  painter.drawLine(QPointF(center_x - outer_radius * 0.94, center_y),
                   QPointF(center_x + outer_radius * 0.94, center_y));
  painter.drawLine(QPointF(center_x, center_y - outer_radius * 0.94),
                   QPointF(center_x, center_y + outer_radius * 0.94));
  painter.drawEllipse(center, outer_radius * 0.5, outer_radius * 0.5);

  // Cardinal markers as quiet dots just inside the rim.
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(96, 112, 128, 120));
  const double dot_r = 2.2;
  const double dot_off = outer_radius - 8.0;
  painter.drawEllipse(center + QPointF(0.0, -dot_off), dot_r, dot_r);
  painter.drawEllipse(center + QPointF(0.0, dot_off), dot_r, dot_r);
  painter.drawEllipse(center + QPointF(-dot_off, 0.0), dot_r, dot_r);
  painter.drawEllipse(center + QPointF(dot_off, 0.0), dot_r, dot_r);

  // Deflection track in Apple accent blue while the stick is pushed.
  if (normalized_speed > 0.02)
  {
    painter.setPen(QPen(QColor(10, 132, 255, 110), 3.0, Qt::SolidLine,
                        Qt::RoundCap));
    painter.drawLine(center, knob);
  }

  // ---- Knob: convex glass cap, lit from the top left.
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(12, 22, 32, 42));
  painter.drawEllipse(knob + QPointF(0.0, 2.5),
                      knob_radius + 2.0, knob_radius + 2.0);
  painter.setBrush(QColor(12, 22, 32, 22));
  painter.drawEllipse(knob + QPointF(0.0, 4.5),
                      knob_radius + 4.0, knob_radius + 4.0);

  QRadialGradient knob_glass(knob - QPointF(6.0, 8.0), knob_radius * 2.0);
  knob_glass.setColorAt(0.0, QColor(255, 255, 255, 252));
  knob_glass.setColorAt(0.55, QColor(238, 243, 248, 246));
  knob_glass.setColorAt(1.0, QColor(204, 214, 225, 240));
  painter.setPen(QPen(frame_valid_ ? QColor(255, 255, 255, 235)
                                  : QColor(255, 96, 96, 235),
                      1.2));
  painter.setBrush(knob_glass);
  painter.drawEllipse(knob, knob_radius, knob_radius);

  // Specular highlight on the cap.
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(255, 255, 255, 150));
  painter.drawEllipse(knob + QPointF(-7.0, -9.0), 7.5, 4.8);

  // ---- Readout, secondary-label styling inside the card's lower band.
  QFont value_font;
  value_font.setPointSizeF(10.5);
  value_font.setLetterSpacing(QFont::AbsoluteSpacing, 0.6);
  value_font.setStyleStrategy(QFont::PreferAntialias);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  painter.setFont(value_font);
  const QString value_text =
      "x " + signedValue(vx) + "   y " + signedValue(vy) + " m/s";
  const QRectF text_rect(10.0, 196.0, 220.0, 26.0);
  painter.setBrush(Qt::NoBrush);
  // Soft white halo keeps the readout legible over any scene now that no
  // panel sits behind it.
  painter.setPen(QColor(255, 255, 255, 130));
  painter.drawText(text_rect.translated(0.0, 1.0), Qt::AlignCenter, value_text);
  painter.setPen(frame_valid_ ? QColor(60, 72, 86, 225)
                             : QColor(200, 60, 60, 235));
  painter.drawText(text_rect, Qt::AlignCenter, value_text);
  painter.end();

  Ogre::HardwarePixelBufferSharedPtr pixel_buffer =
      texture_->getBuffer();
  pixel_buffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox& box = pixel_buffer->getCurrentLock();
  unsigned char* destination = static_cast<unsigned char*>(box.data);
  const int destination_stride = static_cast<int>(box.rowPitch) * 4;
  for (int row = 0; row < kTextureSize; ++row)
  {
    std::memcpy(destination + row * destination_stride,
                image.constScanLine(row), kTextureSize * 4);
  }
  pixel_buffer->unlock();
}

}  // namespace rviz

PLUGINLIB_EXPORT_CLASS(rviz::WorldJoystickDisplay, rviz::Display)
