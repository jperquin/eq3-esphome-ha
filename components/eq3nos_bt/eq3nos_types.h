#pragma once

namespace esphome {
namespace eq3nos_bt {

enum class ConnCommandType {
    SEND_RAW,
    STATUS
};

struct ConnCommand {
    ConnCommandType type;
    uint8_t length;
    uint8_t data[16];   // spazio per un comando EQ3 (ne bastano 1–3 bytes)
};



}  // namespace eq3nos_bt
}  // namespace esphome
