#ifndef sfleta_STACK_H
#define sfleta_STACK_H

#include "sfleta_base.h"

namespace sfleta {
class Stack : public Base {
 public:
  Stack() : Base() {}
  ~Stack();
  size_t Peek() override;
  size_t Pop() override;
};
}  // namespace sfleta
#endif  // sfleta_STACK_H