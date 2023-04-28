#ifndef sfleta_BASE_H
#define sfleta_BASE_H

#include <iostream>

namespace sfleta {
class Base {
 public:
  Base() : size_(0), head_(nullptr), tail_(nullptr) {}
  Base(const Base &) = delete;
  Base &operator=(Base &) = delete;

  virtual ~Base() {
    head_ = nullptr;
    tail_ = nullptr;
  }
  bool Empty() { return size_ == 0; }
  size_t Size() { return size_; }
  void Push(size_t);

  virtual size_t Peek() = 0;
  virtual size_t Pop() = 0;

 protected:
  struct Node {
    size_t data_;
    Node *pNext_;
    Node *pPrev_;
    Node() : data_(), pNext_(nullptr), pPrev_(nullptr) {}
    explicit Node(const size_t &data) : Node() { data_ = data; }
  };

  size_t size_;
  Node *head_;
  Node *tail_;
};
}  // namespace sfleta
#endif  // sfleta_BASE_H
