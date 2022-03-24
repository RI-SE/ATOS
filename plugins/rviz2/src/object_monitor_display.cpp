#include "object_monitor_display.hpp"

#include <iostream>
#include <OgreSceneNode.h>
#include "rclcpp/logging.hpp"

namespace maestro_rviz_plugins {

MonitorDisplay::MonitorDisplay()
: arrow_(nullptr), axes_(nullptr), pose_valid_(false) {
	shape_property_ = new rviz_common::properties::EnumProperty(
		"Shape", "Arrow", "Shape to display the pose as.", this, SLOT(updateShapeChoice()));
	shape_property_->addOption("Arrow", Arrow);
	shape_property_->addOption("Axes", Axes);

	color_property_ = new rviz_common::properties::ColorProperty(
		"Color", QColor(255, 25, 0), "Color to draw the arrow.", this, SLOT(updateColorAndAlpha()));

	alpha_property_ = new rviz_common::properties::FloatProperty(
		"Alpha", 1, "Amount of transparency to apply to the arrow.", this, SLOT(updateColorAndAlpha()));
	alpha_property_->setMin(0);
	alpha_property_->setMax(1);

  shaft_length_property_ = new rviz_common::properties::FloatProperty(
	"Shaft Length", 1, "Length of the arrow's shaft, in meters.",
	this, SLOT(updateArrowGeometry()));

  shaft_radius_property_ = new rviz_common::properties::FloatProperty(
	"Shaft Radius", 0.05f, "Radius of the arrow's shaft, in meters.",
	this, SLOT(updateArrowGeometry()));

  head_length_property_ = new rviz_common::properties::FloatProperty(
	"Head Length", 0.3f, "Length of the arrow's head, in meters.",
	this, SLOT(updateArrowGeometry()));

  head_radius_property_ = new rviz_common::properties::FloatProperty(
	"Head Radius", 0.1f, "Radius of the arrow's head, in meters.",
	this, SLOT(updateArrowGeometry()));

  axes_length_property_ = new rviz_common::properties::FloatProperty(
	"Axes Length", 1, "Length of each axis, in meters.",
	this, SLOT(updateAxisGeometry()));

  axes_radius_property_ = new rviz_common::properties::FloatProperty(
	"Axes Radius", 0.1f, "Radius of each axis, in meters.",
	this, SLOT(updateAxisGeometry()));
}
MonitorDisplay::~MonitorDisplay() = default;

void MonitorDisplay::processMessage(maestro_interfaces::msg::Monitor::ConstSharedPtr msg) {
  
  using rviz_common::validateFloats;
  if (!validateFloats(msg->pose) || !validateFloats(msg->velocity.twist)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  if (
    !context_->getFrameManager()->transform(
      msg->pose.header, msg->pose.pose, position, orientation))
  {
    setMissingTransformToFixedFrame(msg->pose.header.frame_id);
    return;
  }
  setTransformOk();
  pose_valid_ = true;
  updateShapeVisibility();

  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);

  context_->queueRender();
}

void MonitorDisplay::onInitialize() {
  RTDClass::onInitialize();

  arrow_ = std::make_unique<rviz_rendering::Arrow>(
	scene_manager_, scene_node_,
	shaft_length_property_->getFloat(),
	shaft_radius_property_->getFloat(),
	head_length_property_->getFloat(),
	head_radius_property_->getFloat());
  arrow_->setDirection(Ogre::Vector3::UNIT_X);

  axes_ = std::make_unique<rviz_rendering::Axes>(
	scene_manager_, scene_node_,
	axes_length_property_->getFloat(),
	axes_radius_property_->getFloat());

  updateShapeChoice();
  updateColorAndAlpha();
}

void MonitorDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  arrow_->setColor(color);

  context_->queueRender();
}

void MonitorDisplay::updateArrowGeometry()
{
  arrow_->set(
    shaft_length_property_->getFloat(),
    shaft_radius_property_->getFloat(),
    head_length_property_->getFloat(),
    head_radius_property_->getFloat());
  context_->queueRender();
}

void MonitorDisplay::updateAxisGeometry()
{
  axes_->set(
    axes_length_property_->getFloat(),
    axes_radius_property_->getFloat());
  context_->queueRender();
}

void MonitorDisplay::updateShapeChoice()
{
  bool use_arrow = (shape_property_->getOptionInt() == Arrow);

  color_property_->setHidden(!use_arrow);
  alpha_property_->setHidden(!use_arrow);
  shaft_length_property_->setHidden(!use_arrow);
  shaft_radius_property_->setHidden(!use_arrow);
  head_length_property_->setHidden(!use_arrow);
  head_radius_property_->setHidden(!use_arrow);

  axes_length_property_->setHidden(use_arrow);
  axes_radius_property_->setHidden(use_arrow);

  updateShapeVisibility();

  context_->queueRender();
}

void MonitorDisplay::updateShapeVisibility()
{
  if (!pose_valid_) {
    arrow_->getSceneNode()->setVisible(false);
    axes_->getSceneNode()->setVisible(false);
  } else {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    arrow_->getSceneNode()->setVisible(use_arrow);
    axes_->getSceneNode()->setVisible(!use_arrow);
  }
}
void MonitorDisplay::onEnable()
{
  RTDClass::onEnable();
  updateShapeVisibility();
}

void MonitorDisplay::onDisable()
{
  RTDClass::onDisable();
}

void MonitorDisplay::reset()
{
  RTDClass::reset();
  pose_valid_ = false;
  updateShapeVisibility();
}


}  // namespace maestro_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(maestro_rviz_plugins::MonitorDisplay, rviz_common::Display)



