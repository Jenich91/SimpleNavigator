#include "sfleta_base.h"

namespace sfleta {
void Base::Push(size_t value) {
  if (head_ == nullptr) {
    head_ = new Node(value);
    tail_ = head_;
  } else {
    Node *newNode = new Node(value);
    tail_->pNext_ = newNode;
    newNode->pPrev_ = tail_;
    tail_ = newNode;
  }
  size_++;
}
}  // namespace sfleta