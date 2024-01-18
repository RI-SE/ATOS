/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "testobject.hpp"
#include "roschannels/monitorchannel.hpp"
#include "positioning.h"

class RelativeTestObject : public TestObject {
public:
	RelativeTestObject(uint32_t id);
	RelativeTestObject(const RelativeTestObject&) = delete;
	RelativeTestObject(RelativeTestObject&&);

	RelativeTestObject& operator=(const RelativeTestObject&) = delete;
	RelativeTestObject& operator=(RelativeTestObject&&) = default;

private:
    virtual ObjectMonitorType transformCoordinate(const ObjectMonitorType& point,
        const ObjectMonitorType& anchor,
        const bool debug);
    virtual MonitorMessage readMonitorMessage() override;
    
    ROSChannels::Monitor::AnchorSub anchorSub;
    ObjectMonitorType lastAnchorMonr;
    void updateAnchor(const ROSChannels::Monitor::message_type::SharedPtr);
};
