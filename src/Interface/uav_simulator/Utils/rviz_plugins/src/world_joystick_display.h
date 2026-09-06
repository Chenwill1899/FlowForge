#ifndef RVIZ_PLUGINS_WORLD_JOYSTICK_DISPLAY_H
#define RVIZ_PLUGINS_WORLD_JOYSTICK_DISPLAY_H

#include <geometry_msgs/TwistStamped.h>
#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <ros/ros.h>
#include <rviz/display.h>

namespace Ogre
{
class Material;
class Overlay;
class PanelOverlayElement;
class Texture;
}  // namespace Ogre

namespace rviz
{
class FloatProperty;
class RosTopicProperty;
class StringProperty;

/**
 * @brief Screen-space visualization of a world-frame planar velocity command.
 *
 * The display deliberately subscribes to the mapped TwistStamped intent instead
 * of raw sensor_msgs/Joy.  It therefore shows the exact signed, dead-zone
 * filtered command consumed by the navigation stack.  World +X is drawn upward
 * and world +Y is drawn leftward, matching the physical left-stick convention.
 */
class WorldJoystickDisplay : public Display
{
  Q_OBJECT

public:
  WorldJoystickDisplay();
  ~WorldJoystickDisplay() override;

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void updateTopic();
  void updateScale();

private:
  void subscribe();
  void unsubscribe();
  void incomingIntent(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void createOverlay();
  void destroyOverlay();
  void redraw();

  RosTopicProperty* topic_property_;
  StringProperty* expected_frame_property_;
  FloatProperty* full_scale_speed_property_;

  ros::Subscriber intent_sub_;
  geometry_msgs::TwistStamped latest_intent_;
  bool have_intent_;
  bool frame_valid_;
  bool dirty_;

  Ogre::Overlay* overlay_;
  Ogre::PanelOverlayElement* panel_;
  Ogre::TexturePtr texture_;
  Ogre::MaterialPtr material_;

  std::string overlay_name_;
  std::string panel_name_;
  std::string texture_name_;
  std::string material_name_;
};

}  // namespace rviz

#endif  // RVIZ_PLUGINS_WORLD_JOYSTICK_DISPLAY_H
