#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace esphome {
namespace eq3nos_bt {

constexpr uint32_t EQ3_CONNECT_TIMEOUT_MS = 45000;
constexpr uint32_t EQ3_REPLY_TIMEOUT_MS = 15000;
constexpr uint32_t EQ3_APPLICATION_TIMEOUT_MS = 75000;
constexpr uint32_t EQ3_IDENTIFICATION_RETRY_MS = 60000;
constexpr uint32_t EQ3_DISCONNECT_TIMEOUT_MS = 10000;

// Unsigned subtraction keeps deadlines valid when millis() wraps.
inline bool elapsed(uint32_t now, uint32_t since, uint32_t duration) {
  return static_cast<uint32_t>(now - since) >= duration;
}

class SessionDeadline {
 public:
  void start(uint32_t now) { started_ = now; active_ = true; waiting_reply_ = false; }
  void command_sent(uint32_t now) { started_ = now; waiting_reply_ = true; }
  void finish() { active_ = false; }
  bool expired(uint32_t now) const {
    return active_ && elapsed(now, started_, waiting_reply_ ? EQ3_REPLY_TIMEOUT_MS : EQ3_CONNECT_TIMEOUT_MS);
  }
  bool waiting_reply() const { return waiting_reply_; }

 private:
  uint32_t started_{0};
  bool active_{false};
  bool waiting_reply_{false};
};

class IdentificationRetry {
 public:
  void schedule(uint32_t now) { failed_at_ = now; pending_ = true; }
  void cancel() { pending_ = false; }
  bool ready(uint32_t now) const {
    return pending_ && elapsed(now, failed_at_, EQ3_IDENTIFICATION_RETRY_MS);
  }

 private:
  uint32_t failed_at_{0};
  bool pending_{false};
};

// Identification has a four-byte header followed by exactly ten encoded
// serial characters. Never decode trailing bytes left in the GATT value.
inline bool decode_identification(const uint8_t *data, size_t len, std::string &serial, std::string &firmware) {
  if (len < 14 || data[0] != 0x01)
    return false;
  std::string decoded;
  decoded.reserve(10);
  for (size_t i = 4; i < 14; ++i) {
    const int c = static_cast<int>(data[i]) - 0x30;
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')))
      return false;
    decoded.push_back(static_cast<char>(c));
  }
  serial = decoded;
  firmware = std::to_string(data[1]);  // Preserve the existing entity value format.
  return true;
}

}  // namespace eq3nos_bt
}  // namespace esphome
