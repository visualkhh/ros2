#include "test_framework.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

int main() {
  int failed_tests = 0;
  int ran_tests = 0;
  const char* filter = std::getenv("KROSC_TEST_FILTER");
  const auto& tests = krosctest::registry();
  std::cout << "running " << tests.size() << " tests\n";
  for (const auto& test : tests) {
    if (filter != nullptr && test.name.find(filter) == std::string::npos) {
      continue;
    }
    ++ran_tests;
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
  std::cout << "all tests passed (" << ran_tests << " ran)\n";
  return 0;
}
