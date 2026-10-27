#include "eq3nos_session.h"
#include <cassert>
#include <iostream>
#include <vector>
using namespace esphome::eq3nos_bt;
int main() {
  SessionDeadline session;
  session.start(1000);
  // Slow discovery must not consume the reply deadline.
  assert(!session.expired(21000));
  assert(!session.expired(45000));
  session.command_sent(45000);
  assert(!session.expired(59999));
  assert(session.expired(60000));
  session.finish();
  assert(!session.expired(200000));
  session.start(0);
  assert(session.expired(45000));
  session.finish();
  session.start(0xfffffff0u);
  assert(!session.expired(20));
  assert(session.expired(static_cast<uint32_t>(0xfffffff0u + 45000u)));
  IdentificationRetry retry;
  assert(!retry.ready(60000));
  retry.schedule(500);
  assert(!retry.ready(60499));
  assert(retry.ready(60500));
  retry.cancel();
  assert(!retry.ready(100000));
  retry.schedule(0xfffffff0u);
  assert(retry.ready(static_cast<uint32_t>(0xfffffff0u + 60000u)));
  // Identification response with synthetic serial: trailer bytes are not part of the serial.
  std::vector<uint8_t> frame{0x01, 0x92, 0, 0, 0x80, 0x75, 0x81, 0x61,
                           0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x2f};
  std::string serial, firmware;
  assert(decode_identification(frame.data(), frame.size(), serial, firmware));
  assert(serial == "PEQ1234567");
  assert(firmware == "146");
  frame.push_back(0xff);
  frame.push_back(0xff);
  assert(decode_identification(frame.data(), frame.size(), serial, firmware));
  assert(serial == "PEQ1234567");
  assert(!decode_identification(frame.data(), 13, serial, firmware));
  frame[0] = 0x02;
  assert(!decode_identification(frame.data(), frame.size(), serial, firmware));
  frame[0] = 0x01;
  frame[5] = 0xff;
  assert(!decode_identification(frame.data(), frame.size(), serial, firmware));
  assert(serial == "PEQ1234567");
  std::cout << "Session deadlines, retries and identification decoding passed\n";
}
