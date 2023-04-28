#include "sfleta_interface.h"

int main() {
  try {
    sfleta::Interface::GetInstance().Show();
  } catch (std::exception& msg) {
    std::cerr << msg.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
