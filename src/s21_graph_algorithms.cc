#include "sfleta_graph_algorithms.h"

namespace sfleta {
const TsmResult GraphAlgorithms::SolveTravelingSalesmanProblem(Graph &graph) {
  TsmResult result;
  adjacency_matrix_ = graph.GetAdjacencyMatrix();
  size_t graph_size = graph.GetSize();
  bool loop = false;

  InitPheromonesMatrix(graph_size);
  for (size_t step_i = 0; step_i < kColonies; step_i++) {
    for (size_t ant_i = 0; ant_i < kAnts; ant_i++) {
      visted_nodes_.resize(graph_size, true);
      current_node_ = ChoiceRandomNode(0, graph_size - 1);

      ant_path_.push_back(current_node_);
      visted_nodes_[current_node_] = false;

      for (size_t node_i = 0; node_i < graph_size - 1; node_i++) {
        std::map<int, double> not_visited_nodes =
            FindNotVistedNodes(graph_size);

        std::map<int, double> probabilitys;
        double sum_p_all = 0;
        for (const auto &m : not_visited_nodes) {
          sum_p_all += pow(pheromones_matrix_[current_node_][m.first], kAlfa) *
                       pow(1 / m.second, kBeta);
        }

        double range = 0;
        for (const auto &j : not_visited_nodes) {
          double p_ij = 0;
          double value_teta =
              pow(pheromones_matrix_[current_node_][j.first], kAlfa);
          double value_eta = pow(1 / j.second, kBeta);
          p_ij = (value_teta * value_eta) / sum_p_all;
          range += p_ij;
          probabilitys.insert(std::make_pair(j.first, range));
        }
        int next_node = ChoiceNode(probabilitys);

        if (next_node != -1) {
          if (adjacency_matrix_[current_node_][next_node] != 0) {
            ant_path_.push_back(next_node);
            visted_nodes_[next_node] = false;
            current_node_ = next_node;
          }
        } else {
          loop = true;
          break;
        }
      }
      if (loop != true) {
        if (adjacency_matrix_[current_node_][ant_path_.front()] != 0) {
          ant_path_.push_back(ant_path_.front());
          double l_0 = INFINITY;
          if (ant_path_.size() == adjacency_matrix_.size() + 1) {
            l_0 = CalculateLengthAntRoute(ant_path_);
            TsmResult tmp{{ant_path_}, {l_0}};
            ants_routes_.push_back(tmp);
          }
        }
      }
      loop = false;
      visted_nodes_.clear();
      ant_path_.clear();
    }
    UpdatePheromonesMatrix();
    if (ants_routes_.size() != 0) {
      UpdateResult(result);
      ants_routes_.clear();
    }
  }

  if (result.distance == INFINITY) {
    throw std::logic_error("Can't solve Salesman Problem\n");
  }
  return result;
}

void GraphAlgorithms::InitPheromonesMatrix(size_t graph_size) {
  pheromones_matrix_.resize(graph_size, std::vector<double>(graph_size));
  for (size_t i = 0; i < graph_size; i++) {
    for (size_t j = 0; j < graph_size; j++) {
      if (adjacency_matrix_[i][j] != 0) {
        pheromones_matrix_[i][j] = kInitPher;
      }
    }
  }
}

int GraphAlgorithms::ChoiceRandomNode(int iMin, int iMax) {
  std::random_device rd;
  std::default_random_engine engine(rd());
  std::uniform_int_distribution<int> node(iMin, iMax);
  return node(engine);
}

double GraphAlgorithms::ChoiceDoubleRand(double fMin, double fMax) {
  std::random_device rd;
  std::default_random_engine eng(rd());
  std::uniform_real_distribution<double> random_double(fMin, fMax);
  return random_double(eng);
}

std::map<int, double> GraphAlgorithms::FindNotVistedNodes(size_t graph_size) {
  std::map<int, double> list_not_visited_nodes;
  for (size_t j = 0; j < graph_size; j++) {
    if (visted_nodes_[j] && adjacency_matrix_[current_node_][j] != 0) {
      list_not_visited_nodes.insert(
          std::make_pair(j, adjacency_matrix_[current_node_][j]));
    }
  }
  return list_not_visited_nodes;
}

int GraphAlgorithms::ChoiceNode(std::map<int, double> &probabilitys) {
  int next_node;
  if (probabilitys.size() == 1) {
    next_node = probabilitys.begin()->first;
  } else {
    if (probabilitys.size() != 0) {
      double random = ChoiceDoubleRand(0, 1);
      for (auto it = probabilitys.begin(); it != probabilitys.end(); ++it) {
        if (random <= it->second) {
          next_node = it->first;
          break;
        }
      }
    } else {
      next_node = -1;
    }
  }
  return next_node;
}

double GraphAlgorithms::CalculateLengthAntRoute(
    const std::vector<int> &ant_path_) {
  double length_route = 0;
  for (size_t i = 0; i < ant_path_.size() - 1; i++) {
    length_route += adjacency_matrix_[ant_path_[i]][ant_path_[i + 1]];
  }
  return length_route;
}

void GraphAlgorithms::UpdatePheromonesMatrix() {
  for (size_t i = 0; i < pheromones_matrix_.size(); i++) {
    for (size_t j = 0; j < pheromones_matrix_[i].size(); j++) {
      pheromones_matrix_[i][j] = pheromones_matrix_[i][j] * (1 - kP);
    }
  }

  for (size_t i = 0; i < ants_routes_.size(); i++) {
    double delta_teta = kQ / ants_routes_[i].distance;
    for (size_t j = 0; j < ants_routes_[i].vertices.size() - 1; j++) {
      pheromones_matrix_[ants_routes_[i].vertices[j]]
                        [ants_routes_[i].vertices[j + 1]] += delta_teta;
    }
  }
}

void GraphAlgorithms::UpdateResult(TsmResult &result) {
  for (size_t i = 0; i < ants_routes_.size(); i++) {
    if (ants_routes_[i].distance < result.distance) {
      result.vertices = ants_routes_[i].vertices;
      result.distance = ants_routes_[i].distance;
    }
  }
}

std::vector<int> GraphAlgorithms::DepthFirstSearch(Graph &g, int v_index) {
  int n = g.GetSize() - 1;
  --v_index;
  if (v_index < 0 || v_index > n) {
    throw std::out_of_range("vertex_index value - out of range");
  }

  std::vector<int> result;
  auto adjc = g.GetAdjacencyMatrix();

  std::vector<bool> visited(n, false);
  sfleta::Stack stack;

  stack.Push(v_index);

  while (!stack.Empty()) {
    v_index = stack.Pop();
    if (!visited[v_index]) {
      visited[v_index] = true;
      result.push_back(v_index + 1);
    }

    for (int i = n; i >= 0; i--) {
      if (adjc[v_index][i] && !visited[i]) {
        stack.Push(i);
      }
    }
  }
  return result;
}

std::vector<int> GraphAlgorithms::BreadthFirstSearch(Graph &g, int v_index) {
  int n = g.GetSize() - 1;
  --v_index;
  if (v_index < 0 || v_index > n) {
    throw std::out_of_range("vertex_index value - out of range");
  }

  std::vector<int> result;
  auto adjc = g.GetAdjacencyMatrix();

  std::vector<bool> visited(n, false);
  sfleta::Queue queue;
  queue.Push(v_index);

  while (!queue.Empty()) {
    v_index = queue.Pop();
    visited[v_index] = true;
    result.push_back(v_index + 1);

    for (int i = 0; i <= n; ++i) {
      if (adjc[v_index][i] && !visited[i]) {
        queue.Push(i);
        visited[i] = true;
      }
    }
  }
  return result;
}

size_t GraphAlgorithms::GetShortestPathBetweenVertices(Graph &graph,
                                                       int vertex1,
                                                       int vertex2) {
  auto adj_matrix = graph.GetAdjacencyMatrix();
  size_t size = graph.GetSize();

  std::vector visited(size, false);
  std::vector distances(size, kInf);
  size_t min_index(0);

  distances[vertex1 - 1] = 0;
  do {
    size_t min_weight = kInf;
    min_index = kInf;

    for (size_t i = 0; i < size; ++i) {
      if (!visited[i] && distances[i] < min_weight) {
        min_index = i;
        min_weight = distances[i];
      }
    }
    if (min_index != kInf) {
      for (size_t i = 0; i < size; ++i) {
        if (adj_matrix[min_index][i]) {
          distances[i] =
              std::min(distances[i], min_weight + adj_matrix[min_index][i]);
        }
      }
      visited[min_index] = true;
    }
  } while (min_index < kInf);

  return distances[vertex2 - 1];
}

std::vector<std::vector<int>>
GraphAlgorithms::GetShortestPathsBetweenAllVertices(Graph &graph) {
  auto adj_matrix = graph.GetAdjacencyMatrix();
  size_t size = graph.GetSize();
  RefillAdjMatrix(adj_matrix);

  for (size_t k = 0; k < size; ++k) {
    for (size_t i = 0; i < size; ++i) {
      for (size_t j = 0; j < size; ++j) {
        adj_matrix[i][j] =
            std::min(adj_matrix[i][j], adj_matrix[i][k] + adj_matrix[k][j]);
      }
    }
  }
  return adj_matrix;
}

std::vector<std::vector<int>> GraphAlgorithms::GetLeastSpanningTree(
    Graph &graph) {
  auto adj_matrix = graph.GetAdjacencyMatrix();
  size_t size = graph.GetSize();
  RefillAdjMatrix(adj_matrix);

  std::vector visited(size, false);
  std::vector<int> min_weight(size, kInf);
  std::vector min_index(size, -1);

  std::vector least_spanning_tree(size, std::vector<int>(size));

  min_weight[0] = 0;
  for (size_t i = 0; i < size; ++i) {
    int v = kInf;
    for (size_t j = 0; j < size; ++j) {
      if (!visited[j] && (v == kInf || min_weight[j] < min_weight[v])) v = j;
    }
    if (min_weight[v] == kInf) {
      throw std::out_of_range("Несвязный граф!\n");
    }

    visited[v] = true;
    if (min_index[v] != -1) {
      least_spanning_tree[v][min_index[v]] = adj_matrix[v][min_index[v]];
      least_spanning_tree[min_index[v]][v] = adj_matrix[v][min_index[v]];
    }

    for (size_t to = 0; to < size; ++to) {
      if (adj_matrix[v][to] < min_weight[to]) {
        min_weight[to] = adj_matrix[v][to];
        min_index[to] = v;
      }
    }
  }
  return least_spanning_tree;
}

void GraphAlgorithms::RefillAdjMatrix(
    std::vector<std::vector<int>> &adj_matrix) {
  size_t size = adj_matrix.size();
  for (size_t i = 0; i < size; ++i) {
    for (size_t j = 0; j < size; ++j) {
      if (i != j && !adj_matrix[i][j]) adj_matrix[i][j] = kInf;
    }
  }
}

}  // namespace sfleta
