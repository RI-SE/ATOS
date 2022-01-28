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
