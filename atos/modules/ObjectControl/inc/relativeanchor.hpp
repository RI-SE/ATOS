/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "relativetestobject.hpp"
#include "roschannels/monitorchannel.hpp"

class RelativeAnchor : public TestObject {
public:
	RelativeAnchor(uint32_t id);
	RelativeAnchor(const RelativeAnchor&) = delete;
	RelativeAnchor(RelativeAnchor&&);

	RelativeAnchor& operator=(const RelativeAnchor&) = delete;
	RelativeAnchor& operator=(RelativeAnchor&&) = default;

    virtual void publishMonr(const ROSChannels::Monitor::message_type) override;

private:
    ROSChannels::Monitor::AnchorPub anchorPub;
};
