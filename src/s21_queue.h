#ifndef sfleta_QUEUE_H
#define sfleta_QUEUE_H

#include "sfleta_base.h"

namespace sfleta {
class Queue : public Base {
 public:
  Queue() : Base() {}
  ~Queue();
  size_t Peek() override;
  size_t Pop() override;
};
}  // namespace sfleta
#endif  // sfleta_QUEUE_H