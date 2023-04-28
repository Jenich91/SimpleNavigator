#include "sfleta_stack.h"

namespace sfleta {

size_t Stack::Pop() {
  if (Empty()) throw std::logic_error("Error: Pop(): The Stack is empty");
  Node *tmp = tail_;
  size_t value = tmp->data_;
  if (tmp->pPrev_ == nullptr) {
    head_->pNext_ = nullptr;
    head_->pPrev_ = nullptr;
    tail_->pNext_ = nullptr;
    tail_->pPrev_ = nullptr;
    head_ = tail_ = nullptr;
  } else {
    tail_ = tmp->pPrev_;
    tail_->pNext_ = nullptr;
  }
  delete tmp;
  size_--;
  return value;
}

size_t Stack::Peek() {
  if (Empty()) throw std::logic_error("Error: Peek(): The Stack is empty");
  return tail_->data_;
}

Stack::~Stack() {
  while (!Empty()) {
    [[maybe_unused]] auto unused = this->Pop();
  }
}
}  // namespace sfleta