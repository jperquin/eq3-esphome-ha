#pragma once
#include "esphome/components/number/number.h"

namespace esphome {
namespace eq3nos_bt {

class EQ3TargetNumber : public number::Number {
 public:
  void set_parent(class EQ3NOS *parent) { parent_ = parent; }


 protected:
  void control(float value) override;
  class EQ3NOS *parent_{nullptr};
};

}  // namespace eq3nos_bt
}  // namespace esphome
