#include "sfleta_graph.h"

namespace sfleta {
void Graph::LoadGraphFromFile(std::string filename) {
  std::fstream fs;
  fs.open(filename, std::fstream::in | std::fstream::out);
  if (!fs.is_open())
    throw std::invalid_argument(
        "Ошибка при открытии файла c месторасположением: " + filename);

  Clear();

  fs >> size_;
  adjacency_matrix_.resize(size_, std::vector<int>(size_));

  for (size_t i = 0; i < size_; ++i) {
    for (size_t j = 0; j < size_; ++j) {
      fs >> adjacency_matrix_[i][j];
      if (fs.fail()) {
        throw std::invalid_argument("Fail read data for the matrix!");
      }
    }
  }
  fs.close();
}

void Graph::ExportGraphToDot(std::string filename) {
  std::ofstream fout;
  fout.open(filename);
  fout << "graph G {\n";

  for (size_t i = 0; i < size_ - 1; ++i) {
    for (size_t j = i + 1; j < size_; ++j) {
      if (adjacency_matrix_[i][j]) {
        fout << '\t' << i + 1 << "--" << j + 1
             << "[label=" << adjacency_matrix_[i][j] << "]\n";
      }
    }
  }
  fout << "}\n";
  fout.close();
}

void Graph::Clear() {
  if (adjacency_matrix_.size()) {
    for (auto &row : adjacency_matrix_) row.clear();
    adjacency_matrix_.clear();
    size_ = 0;
  }
}

void Graph::Print() {
  for (auto &i : adjacency_matrix_) {
    for (auto &el : i) {
      std::cout << el << " ";
    }
    std::cout << std::endl;
  }
}
}  // namespace sfleta
