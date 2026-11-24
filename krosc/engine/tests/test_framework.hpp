#pragma once

#include <cmath>
#include <exception>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace krosctest {

struct TestCase {
  std::string name;
  std::function<void()> function;
};

inline std::vector<TestCase>& registry() {
  static std::vector<TestCase> tests;
  return tests;
}

inline int& failures() {
  static int count = 0;
  return count;
}

struct Registrar {
  Registrar(std::string name, std::function<void()> function) {
    registry().push_back(TestCase{std::move(name), std::move(function)});
  }
};

inline void fail(const char* file, int line, const std::string& message) {
  ++failures();
  std::cerr << "    " << file << ":" << line << ": " << message << "\n";
}

template <class T>
std::string show(const T& value) {
  std::ostringstream out;
  out << value;
  return out.str();
}

template <class A, class B>
void check_eq(const A& expected, const B& actual, const char* expected_text, const char* actual_text, const char* file,
              int line) {
  if (!(expected == actual)) {
    fail(file, line, std::string(expected_text) + " == " + actual_text + " (expected " + show(expected) + ", got " +
                         show(actual) + ")");
  }
}

inline void check_near(double expected, double actual, double tolerance, const char* file, int line) {
  if (!(std::abs(expected - actual) <= tolerance)) {
    fail(file, line, "near failed (expected " + show(expected) + ", got " + show(actual) + ", tolerance " +
                         show(tolerance) + ")");
  }
}

inline void check_true(bool value, const char* text, const char* file, int line) {
  if (!value) {
    fail(file, line, std::string("check failed: ") + text);
  }
}

}  // namespace krosctest

#define KROSC_TEST(name)                                                        \
  static void name();                                                           \
  static ::krosctest::Registrar krosc_test_registrar_##name(#name, name);        \
  static void name()

#define CHECK(condition) ::krosctest::check_true((condition), #condition, __FILE__, __LINE__)
#define CHECK_EQ(expected, actual) \
  ::krosctest::check_eq((expected), (actual), #expected, #actual, __FILE__, __LINE__)
#define CHECK_NEAR(expected, actual, tolerance) \
  ::krosctest::check_near((expected), (actual), (tolerance), __FILE__, __LINE__)

#define CHECK_THROWS_AS(expression, ExceptionType)                            \
  do {                                                                        \
    bool caught = false;                                                      \
    try {                                                                     \
      (void)(expression);                                                     \
    } catch (const ExceptionType&) {                                          \
      caught = true;                                                          \
    } catch (...) {                                                           \
    }                                                                         \
    ::krosctest::check_true(caught, "expected exception " #ExceptionType " from " #expression, __FILE__, __LINE__); \
  } while (false)

#define CHECK_NOTHROW(expression)                                             \
  do {                                                                        \
    bool threw = false;                                                       \
    try {                                                                     \
      (void)(expression);                                                     \
    } catch (...) {                                                           \
      threw = true;                                                           \
    }                                                                         \
    ::krosctest::check_true(!threw, "unexpected exception from " #expression, __FILE__, __LINE__); \
  } while (false)
