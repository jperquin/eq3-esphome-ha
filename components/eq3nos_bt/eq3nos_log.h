#pragma once
#include "esphome/core/log.h"

#ifndef EQ3_DEVLOG
#define EQ3_DEVLOG 0
#endif

#if EQ3_DEVLOG
  #define EQ3_D(tag, fmt, ...) ESP_LOGD(tag, fmt, ##__VA_ARGS__)
  #define EQ3_V(tag, fmt, ...) ESP_LOGV(tag, fmt, ##__VA_ARGS__)  // verbose
  #define EQ3_DEV(stmt) do { stmt; } while (0)
#else
  #define EQ3_D(tag, fmt, ...) do {} while (0)
  #define EQ3_V(tag, fmt, ...) do {} while (0)
  #define EQ3_DEV(stmt) do {} while (0)
#endif
