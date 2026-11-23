#include "test_framework.hpp"

#include <exception>
#include <iostream>

int main() {
  int failed_tests = 0;
  const auto& tests = krosctest::registry();
  std::cout << "running " << tests.size() << " tests\n";
  for (const auto& test : tests) {
    const int before = krosctest::failures();
    try {
      test.function();
    } catch (const std::exception& error) {
      krosctest::fail(__FILE__, __LINE__, std::string("uncaught exception: ") + error.what());
    } catch (...) {
      krosctest::fail(__FILE__, __LINE__, "uncaught unknown exception");
    }
    if (krosctest::failures() != before) {
      ++failed_tests;
      std::cout << "[FAIL] " << test.name << "\n";
    } else {
      std::cout << "[ OK ] " << test.name << "\n";
    }
  }
  if (failed_tests != 0) {
    std::cerr << failed_tests << " test(s) failed\n";
    return 1;
  }
  std::cout << "all tests passed\n";
  return 0;
}
