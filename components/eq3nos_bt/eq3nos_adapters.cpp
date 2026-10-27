#include "eq3nos_adapters.h"
#include <cmath>


// ---------------- Eq3Serial ----------------
std::vector<uint8_t> Eq3Serial::encode(const std::string& value) {
    std::vector<uint8_t> result;
    result.reserve(value.size());
    for (unsigned char c : value)
        result.push_back(static_cast<uint8_t>(c + 0x30));
    return result;
}

std::string Eq3Serial::decode(const std::vector<uint8_t>& value) {
    std::string result;
    result.reserve(value.size());
    for (uint8_t b : value)
        result.push_back(static_cast<char>(b - 0x30));
    return result;
}
