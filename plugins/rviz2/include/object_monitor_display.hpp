/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <atos_interfaces/msg/monitor.hpp>
#include <rviz_common/ros_topic_display.hpp>
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/objects/axes.hpp"
#include "rviz_rendering/objects/shape.hpp"

namespace atos_rviz_plugins {

class MonitorDisplay : public rviz_common::RosTopicDisplay<atos_interfaces::msg::Monitor> {
	Q_OBJECT
   public:
	enum Shape {
		Arrow,
		Axes,
	};
	MonitorDisplay();

   private:
	void processMessage(atos_interfaces::msg::Monitor::ConstSharedPtr msg);
	void onInitialize() override;
	void reset() override;

	void onEnable() override;
	void onDisable() override;

	~MonitorDisplay() override;

   private Q_SLOTS:

	void updateShapeVisibility();
	void updateShapeChoice();
	void updateColorAndAlpha();
	void updateAxisGeometry();
	void updateArrowGeometry();

   private:
	rviz_common::properties::EnumProperty* shape_property_;

	rviz_common::properties::ColorProperty* color_property_;
	rviz_common::properties::FloatProperty* alpha_property_;

	rviz_common::properties::FloatProperty* head_radius_property_;
	rviz_common::properties::FloatProperty* head_length_property_;
	rviz_common::properties::FloatProperty* shaft_radius_property_;
	rviz_common::properties::FloatProperty* shaft_length_property_;

	rviz_common::properties::FloatProperty* axes_length_property_;
	rviz_common::properties::FloatProperty* axes_radius_property_;

	std::unique_ptr<rviz_rendering::Arrow> arrow_;
	std::unique_ptr<rviz_rendering::Axes> axes_;

	bool pose_valid_;
};

}  // namespace atos_rviz_plugins
