#include "sfleta_queue.h"

namespace sfleta {
size_t Queue::Peek() {
  if (Empty()) throw std::logic_error("Error: Peek(): The Queue is empty");
  return head_->data_;
}

size_t Queue::Pop() {
  if (Empty()) throw std::logic_error("Error: Pop(): The Queue is empty");

  size_t value = 0;
  if (head_ != nullptr) {
    value = head_->data_;
    Node *tmp = head_;
    head_ = head_->pNext_;
    delete tmp;
    if (head_ != nullptr) head_->pPrev_ = nullptr;
    size_--;
  }
  return value;
}

Queue::~Queue() {
  while (!Empty()) {
    [[maybe_unused]] auto unused = this->Pop();
  }
}
}  // namespace sfleta