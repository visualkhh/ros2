#include <string>
#include <vector>

#include "krosc/context.hpp"
#include "krosc/node.hpp"
#include "krosc/parameter.hpp"

#include "test_framework.hpp"

KROSC_TEST(parameter_defaults_type_and_validator) {
  krosc::ParameterManager manager;
  auto value = manager.declare<double>("node", "speed", 1.5, [](double item) { return item > 0.0; });
  CHECK_NEAR(1.5, value.value(), 1e-12);
  value.set(2.0);
  CHECK_NEAR(2.0, value.value(), 1e-12);
  CHECK_THROWS_AS(value.set(-1.0), krosc::ParameterRejectedError);
  CHECK_THROWS_AS(manager.set("node", "speed", std::int64_t{3}), krosc::ParameterTypeError);
  CHECK_THROWS_AS(manager.get("node", "missing"), krosc::ParameterNotDeclaredError);
  CHECK_THROWS_AS(manager.declare<double>("node", "speed", 2.0), krosc::ParameterAlreadyDeclaredError);
}

KROSC_TEST(parameter_overrides_precedence) {
  krosc::ParameterManager::Overrides overrides;
  overrides["node"]["value"] = static_cast<std::int64_t>(7);
  krosc::ParameterManager manager(overrides);
  manager.set_node_overrides("node", {{"value", static_cast<std::int64_t>(5)}});
  auto value = manager.declare<std::int64_t>("node", "value", std::int64_t{1});
  CHECK_EQ(std::int64_t{7}, value.value());
}

KROSC_TEST(parameter_on_change_and_events) {
  krosc::ParameterManager manager;
  std::vector<krosc::ParameterEvent> events;
  manager.set_event_callback([&](const krosc::ParameterEvent& event) { events.push_back(event); });
  double observed = 0.0;
  auto value = manager.declare<double>(
      "node", "value", 1.0, {}, [&](double next) { observed = next; });
  value.set(2.5);
  CHECK_NEAR(2.5, observed, 1e-12);
  CHECK_EQ(2u, events.size());
  if (events.size() == 2) {
    CHECK(events[0].declared);
    CHECK(!events[1].declared);
    CHECK(events[1].old_value.has_value());
  }
}

KROSC_TEST(parameter_vectors_and_strings) {
  krosc::ParameterManager manager;
  auto diagonal = manager.declare<std::vector<double>>("node", "diag", {1.0, 2.0, 3.0});
  CHECK_EQ(3u, diagonal.value().size());
  auto text = manager.declare<std::string>("node", "frame", "odom");
  CHECK_EQ(std::string("odom"), text.value());
  CHECK_THROWS_AS(manager.set("node", "diag", std::vector<std::int64_t>{1}), krosc::ParameterTypeError);
}

KROSC_TEST(node_accepts_int_parameters) {
  krosc::Context context;
  krosc::Node node(context, "int_parameter");
  auto count = node.declare_parameter("count", 3);
  CHECK_EQ(std::int64_t{3}, count.value());
  node.set_parameter("count", 7);
  CHECK_EQ(std::int64_t{7}, count.value());
  CHECK_THROWS_AS(node.set_parameter("count", 7.5), krosc::ParameterTypeError);
}

KROSC_TEST(parameter_unused_overrides) {
  krosc::ParameterManager::Overrides overrides;
  overrides["node"]["typo"] = true;
  overrides["node"]["used"] = static_cast<std::int64_t>(1);
  krosc::ParameterManager manager(overrides);
  (void)manager.declare<std::int64_t>("node", "used", std::int64_t{0});
  const auto unused = manager.unused_overrides();
  CHECK_EQ(1u, unused.size());
  if (!unused.empty()) {
    CHECK_EQ(std::string("node.typo"), unused[0]);
  }
}
