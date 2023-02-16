/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef OBJECTDATA_HPP
#define OBJECTDATA_HPP
#include "trajectory.hpp"

class ObjectConfiguration
{
public:
	ObjectConfiguration();
	ObjectConfiguration(const std::string &fromFile);
	Trajectory trajectory;
	in_addr_t ip = 0;
	uint32_t id = 0;

	void initializeFromFile(const std::string &fileName);
};

#endif // OBJECTDATA_HPP
