/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef	TRAJFILEHANDLER_H
#define	TRAJFILEHANDLER_H

#include <vector>
#include <string>
#include <sstream>
#include "util.h"

int parseTraj(std::string line, std::vector<char>& buffer);


#endif