/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "module.hpp"
#include "roschannels/commandchannels.hpp"


class IntegrationTesting : public Module {

	public:
		IntegrationTesting();
		~IntegrationTesting();

	private:
	  static inline std::string const moduleName = "integration_testing";

};
