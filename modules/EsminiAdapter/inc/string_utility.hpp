#pragma once
#include <string>
#include <vector>
#include <sstream>

// Function to split a string into a vector of strings by a delimiter
void split (const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}