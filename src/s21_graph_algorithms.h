#ifndef sfleta_GRAPH_ALGORITHMS
#define sfleta_GRAPH_ALGORITHMS

#include <map>
#include <random>

#include "sfleta_graph.h"
#include "sfleta_queue.h"
#include "sfleta_stack.h"

namespace sfleta {
static const constexpr size_t kInf = 1e8;
const double kAlfa = 1.0;
const double kBeta = 1.0;
const double kQ = 4.0;
const double kP = 0.5;
const double kInitPher = 0.2;
const size_t kColonies = 40;
const size_t kAnts = 200;

struct TsmResult {
  std::vector<int> vertices;
  double distance = INFINITY;
};

class GraphAlgorithms {
 public:
  GraphAlgorithms() = default;
  ~GraphAlgorithms() {}

  std::vector<int> DepthFirstSearch(Graph &graph, int vertex);
  std::vector<int> BreadthFirstSearch(Graph &graph, int vertex);
  size_t GetShortestPathBetweenVertices(Graph &graph, int vertex1, int vertex2);
  auto GetShortestPathsBetweenAllVertices(Graph &)
      -> std::vector<std::vector<int>>;
  auto GetLeastSpanningTree(Graph &) -> std::vector<std::vector<int>>;
  const TsmResult SolveTravelingSalesmanProblem(Graph &graph);

 private:
  std::vector<std::vector<double>> pheromones_matrix_;
  std::vector<std::vector<int>> adjacency_matrix_;
  std::vector<bool> visted_nodes_;
  std::vector<int> ant_path_;
  int current_node_;
  std::vector<TsmResult> ants_routes_;

  void RefillAdjMatrix(std::vector<std::vector<int>> &);

  void InitPheromonesMatrix(size_t graph_size);
  int ChoiceRandomNode(int iMin, int iMax);
  double ChoiceDoubleRand(double fMin, double fMax);
  std::map<int, double> FindNotVistedNodes(size_t graph_size);
  int ChoiceNode(std::map<int, double> &probabilitys);
  double CalculateLengthAntRoute(const std::vector<int> &ant_path_);
  void UpdatePheromonesMatrix();
  void UpdateResult(TsmResult &result);
};
}  // namespace sfleta
#endif  // sfleta_GRAPH_ALGORITHMS