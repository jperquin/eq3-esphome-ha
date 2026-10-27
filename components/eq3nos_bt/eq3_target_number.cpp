#include "eq3_target_number.h"
#include "eq3nos_bt.h"

namespace esphome {
namespace eq3nos_bt {

void EQ3TargetNumber::control(float value) {
  if (parent_ != nullptr) {
    parent_->set_target_temperature(value);
  }
}

}  // namespace eq3nos_bt
}  // namespace esphome
