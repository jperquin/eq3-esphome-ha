#pragma once

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace eq3nos_bt {

class EQ3NOS;

/// @brief Home Assistant entity: Mode selector.
/// @ingroup eq3_component
class EQ3ModeSelect : public select::Select, public Component {
 public:
  void set_parent(EQ3NOS *parent) { parent_ = parent; }

 protected:
  void control(const std::string &value) override;
  EQ3NOS *parent_{nullptr};
};

/// @brief Home Assistant entity: Lock switch.
/// @ingroup eq3_component
class EQ3LockSwitch : public esphome::switch_::Switch {
 public:
  void set_parent(EQ3NOS *parent) { parent_ = parent; }
  void write_state(bool state) override;

 private:
  EQ3NOS *parent_{nullptr};
};

/// @brief Home Assistant entity: Boost switch.
/// @ingroup eq3_component
class EQ3BoostSwitch : public esphome::switch_::Switch {
 public:
  void set_parent(EQ3NOS *parent) { parent_ = parent; }
  void write_state(bool state) override;

 private:
  EQ3NOS *parent_{nullptr};
};

/// @brief Home Assistant entity: Off switch.
/// @ingroup eq3_component
class EQ3OffSwitch : public esphome::switch_::Switch {
 public:
  void set_parent(EQ3NOS *parent) { parent_ = parent; }
  void write_state(bool state) override;

 private:
  EQ3NOS *parent_{nullptr};
};

}  // namespace eq3nos_bt
}  // namespace esphome
