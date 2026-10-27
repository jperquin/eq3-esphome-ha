#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <ctime>


// -----------------------------------------------
//  Eq3Serial
// -----------------------------------------------
class Eq3Serial {
public:
    static std::vector<uint8_t> encode(const std::string& value);
    static std::string decode(const std::vector<uint8_t>& value);
};



// -----------------------------------------------
//  Eq3Time
// -----------------------------------------------
class Eq3Time {
public:
    static std::vector<uint8_t> encode(const std::tm& value);
    static std::tm decode(const std::vector<uint8_t>& value);
};

