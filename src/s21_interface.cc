#include "sfleta_interface.h"

namespace sfleta {
void Interface::Show() {
  int input = -1;
  std::stringstream message("");
  message << "Выберите действие:\n"
          << "\t1 - Загрузка исходного графа из файла\n"
          << "\t0 - Выход\n";
  MenuSelectionWrapper(0, 1, input, message.str());

  while (input) {
    system("clear");
    message.str("");
    message
        << "Выберите действие:\n"
        << "\t1 - Загрузка нового графа из файла\n"
        << "\t2 - Обход графа в ширину\n"
        << "\t3 - Обход графа в глубину\n"
        << "\t4 - Поиск кратчайшего пути между произвольными двумя вершинами\n"
        << "\t5 - Поиск кратчайших путей между всеми парами вершин в графе\n"
        << "\t6 - Поиск минимального остовного дерева в графе\n"
        << "\t7 - Решение задачи комивояжера\n"
        << "\t0 - Выход\n";

    MenuSelectionWrapper(0, 7, input, message.str());
  }
}

void Interface::MenuSelectionWrapper(int low, int high, int &input,
                                     const std::string &message) {
  while (true) {
    std::cout << message;
    std::cin >> input;
    if (std::cin.fail()) {
      WrongInputAtention();
    } else {
      if (!InRange(low, high, input)) {
        std::cout << "Неверный ввод, введите еще раз\n";
      } else if (input == 0) {
        throw std::logic_error("Выходим...");
      } else {
        TryHandleActions(input);
        std::cout << "Для продолжения нажмите Enter\n";

        std::cin.sync();
        std::cin.ignore(INT_MAX, '\n');
        break;
      }
    }
  }
}

void Interface::TryHandleActions(int input) {
  int exit_flag = -1;
  do {
    try {
      Actions(input);
      break;
    } catch (const std::exception &e) {
      std::cout << e.what() << std::endl;

      while (true) {
        std::cout << "Выберите действие:\n"
                  << "\t1 - Для продолжения\n"
                  << "\t0 - Выход\n";
        std::cin >> exit_flag;
        if (std::cin.fail() || !InRange(0, 1, exit_flag)) {
          WrongInputAtention();
          continue;
        } else if (exit_flag == 0) {
          throw std::logic_error("Выходим...");
        } else if (exit_flag == 1) {
          break;
        }
      }
    }
  } while (true);
}

void Interface::WrongInputAtention() {
  std::cout << "Неверный ввод, недопустимое значение или введено не число\n";
  std::cin.clear();
  std::cin.ignore(INT_MAX, '\n');
}

bool Interface::InRange(int low, int high, int x) {
  return ((x - low) <= (high - low));
}

void Interface::Actions(int action) {
  if (action == 1) {
    std::cout << "Введите путь до файла и его имя\n";
    std::string file_name;
    std::cin >> file_name;

    graph_->LoadGraphFromFile(file_name);
    graph_->Print();
  } else if (graph_->GetSize()) {
    if (action == 2 || action == 3) {
      std::cout << "Введите начальную вершину\n";
      int start_vertex;
      std::cin >> start_vertex;
      if (IsValid(start_vertex)) {
        if (action == 2) {
          auto result = algorithms_->BreadthFirstSearch(*graph_, start_vertex);
          PrintResult(result);
        } else {
          auto result = algorithms_->DepthFirstSearch(*graph_, start_vertex);
          PrintResult(result);
        }
      } else
        std::cout << "Неверная вершина\n";

    } else if (action == 4) {
      std::cout << "Введите начальную и конечную вершины\n";
      int start_vertex, end_vertex;
      std::cin >> start_vertex >> end_vertex;
      if (IsValid(start_vertex, end_vertex)) {
        auto result = algorithms_->GetShortestPathBetweenVertices(
            *graph_, start_vertex, end_vertex);
        std::cout << "Наименьшее расстояние между точками " << start_vertex
                  << " и " << end_vertex << " равно " << result << std::endl;
      } else
        std::cout << "Неверные вершины\n";

    } else if (action == 5) {
      auto result = algorithms_->GetShortestPathsBetweenAllVertices(*graph_);
      std::cout << "Матрица кратчайших путей:" << std::endl;
      PrintResult(result);
    } else if (action == 6) {
      auto result = algorithms_->GetLeastSpanningTree(*graph_);
      std::cout << "Матрица смежности для минимального остовного дерева:"
                << std::endl;
      PrintResult(result);
    } else if (action == 7) {
      auto result = algorithms_->SolveTravelingSalesmanProblem(*graph_);
      std::cout << "Решение задачи коммивояжера:" << std::endl;
      PrintResult(result);
    }

  } else {
    std::cout << "Граф не задан\n";
  }
}

bool Interface::IsValid(size_t start_vertex, size_t end_vertex) {
  return start_vertex > 0 && end_vertex > 0 &&
         start_vertex <= graph_->GetSize() && end_vertex <= graph_->GetSize();
}

void Interface::PrintResult(std::vector<std::vector<int>> result) {
  for (auto &rows : result) {
    for (auto &value : rows) std::cout << value << ' ';
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void Interface::PrintResult(std::vector<int> result) {
  for (auto &&value : result) {
    std::cout << value << ' ';
  }
  std::cout << std::endl;
}

void Interface::PrintResult(const TsmResult &result) {
  auto [vertices, distance] = (result);
  std::cout << "Выгодный (короткий) маршрут:" << std::endl;
  for (auto &value : vertices) std::cout << value + 1 << ' ';
  std::cout << "\nДлина этого маршрута: " << distance << std::endl;
}

}  // namespace sfleta
