#pragma once
#include <climits>
#include <sstream>

#include "sfleta_graph_algorithms.h"

namespace sfleta {
class Interface {
 public:
  ~Interface() {
    delete graph_;
    delete algorithms_;
  }
  Interface(const Interface &) = delete;
  Interface(const Interface &&) = delete;
  Interface &operator=(const Interface &) = delete;
  Interface &operator=(const Interface &&) = delete;

  void Show();
  static Interface &GetInstance() {
    static Interface instance;
    return instance;
  }

 private:
  Interface() : graph_{new Graph}, algorithms_{new GraphAlgorithms} {}
  bool IsValid(size_t start_vertex = 1, size_t end_vertex = 1);
  void PrintResult(std::vector<std::vector<int>>);
  void PrintResult(std::vector<int>);
  void PrintResult(const TsmResult &);
  void WrongInputAtention();
  bool InRange(int, int, int);
  void TryHandleActions(int);
  void MenuSelectionWrapper(int, int, int &, const std::string &);
  void Actions(int);
  Graph *graph_;
  GraphAlgorithms *algorithms_;
};
}  // namespace sfleta