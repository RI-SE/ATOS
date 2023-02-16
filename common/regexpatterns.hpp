/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef REGEXPATTERNS_H
#define REGEXPATTERNS_H
#include <iostream>

namespace RegexPatterns {
const std::string intPattern = "[0-9]+";
const std::string floatPattern = "[-+]?[0-9]*\\.?[0-9]+";
const std::string namePattern = "[a-zA-Z0-9\\._-]+";
const std::string versionPattern = "v?(" + intPattern + ")\\.?(" + intPattern + ")?\\.?(" + intPattern + ")?";
}


#endif
