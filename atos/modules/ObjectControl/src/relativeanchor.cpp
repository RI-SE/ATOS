/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "testobject.hpp"
#include <cstdint>

RelativeAnchor::RelativeAnchor(uint32_t id) : 
    TestObject(id),
	anchorPub(*this)
{
}

RelativeAnchor::RelativeAnchor(RelativeAnchor&& other) : 
	TestObject(std::move(other)),
	anchorPub(std::move(other.anchorPub))
{
}

RelativeAnchor::publishMonr(const ROSChannels::Monitor::message_type monr) {
	monrPub.publish(monr);
	anchorPub.publish(monr);
}