#include <queue>
#include <stack>

#include "gtest/gtest.h"
#include "sfleta_graph_algorithms.h"

void IsEqualQueue(sfleta::Queue& my, std::queue<size_t>& origin) {
  ASSERT_EQ(my.Size(), origin.size());
  while (!my.Empty()) {
    auto value = my.Pop();
    auto value2 = origin.front();
    origin.pop();
    ASSERT_EQ(value, value2);
  }
  ASSERT_EQ(my.Empty(), origin.empty());
}

void IsEqualStack(sfleta::Stack& my, std::stack<size_t>& origin) {
  ASSERT_EQ(my.Size(), origin.size());
  while (!my.Empty()) {
    auto value = my.Pop();
    auto value2 = origin.top();
    origin.pop();
    ASSERT_EQ(value, value2);
  }
  ASSERT_EQ(my.Empty(), origin.empty());
}

void IsEqualShortestPath(sfleta::Graph& checking_graph,
                         sfleta::Graph& checked_graph) {
  sfleta::GraphAlgorithms alg;
  ASSERT_EQ(checking_graph.GetSize(), checked_graph.GetSize());

  for (size_t i = 0; i < checking_graph.GetSize(); ++i)
    for (size_t j = 0; j < checked_graph.GetSize(); ++j)
      ASSERT_EQ(
          alg.GetShortestPathBetweenVertices(checking_graph, i + 1, j + 1),
          checked_graph.GetAdjacencyMatrix()[i][j]);
}

TEST(queue, queue_1) {
  sfleta::Queue my;
  std::queue<size_t> origin;

  my.Push(10);
  my.Push(9);
  my.Push(8);
  my.Push(7);
  my.Push(6);

  origin.push(10);
  origin.push(9);
  origin.push(8);
  origin.push(7);
  origin.push(6);

  IsEqualQueue(my, origin);
}

TEST(queue, queue_2) {
  sfleta::Queue my;
  std::queue<size_t> origin;

  my.Push(77);
  my.Push(108);
  my.Push(10006);

  origin.push(77);
  origin.push(108);
  origin.push(10006);

  IsEqualQueue(my, origin);
}

TEST(queue, queue_3) {
  sfleta::Queue my;
  std::queue<size_t> origin;

  my.Push(77);
  my.Pop();
  my.Push(108);
  my.Pop();
  my.Push(10006);

  origin.push(77);
  origin.pop();
  origin.push(108);
  origin.pop();
  origin.push(10006);

  ASSERT_TRUE(my.Peek() == origin.front());
}

TEST(stack, stack_1) {
  sfleta::Stack my;
  std::stack<size_t> origin;

  my.Push(10);
  my.Push(9);
  my.Push(8);
  my.Push(7);
  my.Push(6);

  origin.push(10);
  origin.push(9);
  origin.push(8);
  origin.push(7);
  origin.push(6);

  IsEqualStack(my, origin);
}

TEST(stack, stack_2) {
  sfleta::Stack my;
  std::stack<size_t> origin;

  my.Push(77);
  my.Push(108);
  my.Push(10006);

  origin.push(77);
  origin.push(108);
  origin.push(10006);

  IsEqualStack(my, origin);
}

TEST(stack, stack_3) {
  sfleta::Stack my;
  std::stack<size_t> origin;

  my.Push(77);
  my.Pop();
  my.Push(108);
  my.Pop();
  my.Push(10006);

  origin.push(77);
  origin.pop();
  origin.push(108);
  origin.pop();
  origin.push(10006);

  ASSERT_TRUE(my.Peek() == origin.top());
}

TEST(Export, test) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");
  checking_graph.ExportGraphToDot("./export/graph_1_test.dot");
  checking_graph.Print();

  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");
  checking_graph.ExportGraphToDot("./export/graph_2_test.dot");
}

TEST(getShortestPathBetweenVertices, test_1) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_1_shortest_path.txt");

  IsEqualShortestPath(checking_graph, checked_graph);
}

TEST(getShortestPathBetweenVertices, test_2) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_2_shortest_path.txt");

  IsEqualShortestPath(checking_graph, checked_graph);
}

TEST(getShortestPathBetweenVertices, test_3) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_3_shortest_path.txt");

  IsEqualShortestPath(checking_graph, checked_graph);
}

TEST(getShortestPathsBetweenAllVertices, test_1) {
  sfleta::GraphAlgorithms alg;
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_1_shortest_path.txt");

  ASSERT_TRUE(alg.GetShortestPathsBetweenAllVertices(checking_graph) ==
              checked_graph.GetAdjacencyMatrix());
}

TEST(getShortestPathsBetweenAllVertices, test_2) {
  sfleta::GraphAlgorithms alg;
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_2_shortest_path.txt");

  ASSERT_TRUE(alg.GetShortestPathsBetweenAllVertices(checking_graph) ==
              checked_graph.GetAdjacencyMatrix());
}

TEST(getShortestPathsBetweenAllVertices, test_3) {
  sfleta::GraphAlgorithms alg;
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_3_shortest_path.txt");

  ASSERT_TRUE(alg.GetShortestPathsBetweenAllVertices(checking_graph) ==
              checked_graph.GetAdjacencyMatrix());
}

TEST(GetLeastSpanningTree, LST_1) {
  sfleta::GraphAlgorithms alg;
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");
  auto LST = alg.GetLeastSpanningTree(checking_graph);

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_1_LST.txt");
  ASSERT_TRUE(LST == checked_graph.GetAdjacencyMatrix());
}

TEST(GetLeastSpanningTree, LST_2) {
  sfleta::GraphAlgorithms alg;
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");
  auto LST = alg.GetLeastSpanningTree(checking_graph);

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_2_LST.txt");
  ASSERT_TRUE(LST == checked_graph.GetAdjacencyMatrix());
}

TEST(GetLeastSpanningTree, LST_3) {
  sfleta::GraphAlgorithms alg;
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");
  auto LST = alg.GetLeastSpanningTree(checking_graph);

  sfleta::Graph checked_graph;
  checked_graph.LoadGraphFromFile("./examples/graph_3_LST.txt");
  ASSERT_TRUE(LST == checked_graph.GetAdjacencyMatrix());
}

TEST(DepthFirstSearch, test_1) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 1);

  std::vector<int> reference{1, 2, 3, 4, 5, 6};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_2) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 3);

  std::vector<int> reference{3, 1, 2, 5, 4, 6};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_3) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 5);

  std::vector<int> reference{5, 1, 2, 3, 4, 6};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_4) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 6);

  std::vector<int> reference{6, 1, 2, 3, 4, 5};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_5) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 1);

  std::vector<int> reference{1, 2, 3, 4, 5, 6, 7, 8, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_6) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 3);

  std::vector<int> reference{3, 1, 2, 4, 5, 6, 7, 8, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_7) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 5);

  std::vector<int> reference{5, 1, 2, 3, 4, 6, 7, 8, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_8) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 8);

  std::vector<int> reference{8, 1, 2, 3, 4, 5, 6, 7, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_9) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 1);

  std::vector<int> reference{1, 4, 3, 2, 5};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_10) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 2);

  std::vector<int> reference{2, 3, 4, 1, 5};

  ASSERT_TRUE(funcOut == reference);
}

TEST(DepthFirstSearch, test_11) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.DepthFirstSearch(checking_graph, 4);

  std::vector<int> reference{4, 1, 5, 2, 3};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_1) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 1);

  std::vector<int> reference{1, 2, 3, 4, 5, 6};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_2) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 2);

  std::vector<int> reference{2, 1, 3, 5, 6, 4};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_3) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 3);

  std::vector<int> reference{3, 1, 2, 4, 5, 6};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_4) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_1.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 5);

  std::vector<int> reference{5, 1, 2, 3, 4, 6};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_5) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 1);

  std::vector<int> reference{1, 2, 3, 4, 5, 6, 7, 8, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_6) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 3);

  std::vector<int> reference{3, 1, 2, 4, 5, 6, 7, 8, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_7) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 5);

  std::vector<int> reference{5, 1, 2, 3, 4, 6, 7, 8, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_8) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_2.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 8);

  std::vector<int> reference{8, 1, 2, 3, 4, 5, 6, 7, 9};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_9) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 1);

  std::vector<int> reference{1, 4, 5, 3, 2};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_10) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 3);

  std::vector<int> reference{3, 2, 4, 5, 1};

  ASSERT_TRUE(funcOut == reference);
}

TEST(BreadthFirstSearch, test_11) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/graph_3.txt");

  sfleta::GraphAlgorithms GA;
  std::vector<int> funcOut = GA.BreadthFirstSearch(checking_graph, 4);

  std::vector<int> reference{4, 1, 3, 5, 2};

  ASSERT_TRUE(funcOut == reference);
}

TEST(SolveTravelingSalesmanProblem, test_1) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/sale_1.txt");

  sfleta::GraphAlgorithms alg;
  const sfleta::TsmResult res = alg.SolveTravelingSalesmanProblem(checking_graph);

  double answer = 253;
  ASSERT_EQ(answer, res.distance);
}

TEST(SolveTravelingSalesmanProblem, test_2) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/sale_2.txt");

  sfleta::GraphAlgorithms alg;
  const sfleta::TsmResult res = alg.SolveTravelingSalesmanProblem(checking_graph);

  double answer = 127;
  ASSERT_EQ(answer, res.distance);
}

TEST(SolveTravelingSalesmanProblem, test_3) {
  sfleta::Graph checking_graph;
  checking_graph.LoadGraphFromFile("./examples/sale_3_err.txt");

  sfleta::GraphAlgorithms alg;
  const sfleta::TsmResult res = alg.SolveTravelingSalesmanProblem(checking_graph);

  double answer = 69;
  ASSERT_EQ(answer, res.distance);
}

// TEST(SolveTravelingSalesmanProblem, test_4) {
//     sfleta::Graph checking_graph;
//     checking_graph.LoadGraphFromFile("./examples/sale_4_err.txt");

//     sfleta::GraphAlgorithms alg;
//     EXPECT_THROW(alg.SolveTravelingSalesmanProblem(checking_graph),
//     std::exception);
// }

// TEST(SolveTravelingSalesmanProblem, test_5) {
//     sfleta::Graph checking_graph;
//     checking_graph.LoadGraphFromFile("./examples/sale_5_err.txt");

//     sfleta::GraphAlgorithms alg;
//     EXPECT_THROW(alg.SolveTravelingSalesmanProblem(checking_graph),
//     std::exception);
// }

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}