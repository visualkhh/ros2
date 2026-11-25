#include "krosc/node.hpp"

#include <algorithm>
#include <exception>
#include <optional>

#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/parameter_rpc.hpp"

#include <atomic>
#include <stdexcept>
#include <utility>

namespace krosc {

namespace {
std::atomic<std::uint64_t> next_node_id{1};
}

Node::Node(Context& context, std::string name, std::map<std::string, ParamValue> parameters)
    : context_(context), name_(name.empty() ? "node_" + std::to_string(next_node_id.fetch_add(1)) : std::move(name)) {
  context_.register_node(name_);
  if (!parameters.empty()) {
    context_.parameters().set_node_overrides(name_, std::move(parameters));
  }
  log_level_ = context_.parameters().declare<std::string>(
      name_, "log_level", "INFO",
      [](const std::string& level) { return parse_log_level(level).has_value(); });
  logger_ = std::make_unique<NodeLogger>(
      name_, [this](const msgs::Log& log) { publish(ROSOUT_TOPIC_NAME, log); },
      [this] { return context_.now(); },
      [this] {
        const auto level = parse_log_level(log_level_.value());
        return level.value_or(LogLevel::Info);
      });
  create_parameter_services();
}

void Node::create_parameter_services() {
  using namespace msgs::rcl_interfaces;

  create_service<GetParametersRequest, GetParametersResponse>(
      name_ + "/get_parameters", [this](const GetParametersRequest& request) {
        GetParametersResponse response;
        response.values.reserve(request.names.size());
        for (const auto& name : request.names) {
          try {
            response.values.push_back(parameter_value_message(context_.parameters().get(name_, name)));
          } catch (const std::exception&) {
            response.values.push_back(ParameterValue{});
          }
        }
        return response;
      });

  create_service<SetParametersRequest, SetParametersResponse>(
      name_ + "/set_parameters", [this](const SetParametersRequest& request) {
        SetParametersResponse response;
        response.results.reserve(request.parameters.size());
        for (const auto& parameter : request.parameters) {
          SetParametersResult result;
          try {
            context_.parameters().set(name_, parameter.name,
                                      param_value_from_message(parameter.value));
            result.successful = true;
          } catch (const std::exception& error) {
            result.reason = error.what();
          }
          response.results.push_back(std::move(result));
        }
        return response;
      });

  create_service<SetParametersRequest, SetParametersResponse>(
      name_ + "/set_parameters_atomically", [this](const SetParametersRequest& request) {
        SetParametersResponse response;
        std::vector<std::optional<ParamValue>> previous;
        previous.reserve(request.parameters.size());
        for (const auto& parameter : request.parameters) {
          try {
            previous.push_back(context_.parameters().get(name_, parameter.name));
          } catch (const std::exception&) {
            previous.push_back(std::nullopt);
          }
        }
        std::size_t applied = 0;
        std::string failure;
        for (const auto& parameter : request.parameters) {
          try {
            context_.parameters().set(name_, parameter.name,
                                      param_value_from_message(parameter.value));
            ++applied;
          } catch (const std::exception& error) {
            failure = error.what();
            break;
          }
        }
        if (!failure.empty()) {
          // 실패하면 앞서 적용한 값을 되돌린다 (on_change가 다시 불릴 수 있다).
          for (std::size_t i = 0; i < applied; ++i) {
            if (!previous[i]) {
              continue;
            }
            try {
              context_.parameters().set(name_, request.parameters[i].name, *previous[i]);
            } catch (const std::exception&) {
            }
          }
          for (const auto& parameter : request.parameters) {
            (void)parameter;
            SetParametersResult result;
            result.reason = failure;
            response.results.push_back(std::move(result));
          }
          return response;
        }
        for (const auto& parameter : request.parameters) {
          (void)parameter;
          SetParametersResult result;
          result.successful = true;
          response.results.push_back(std::move(result));
        }
        return response;
      });

  create_service<ListParametersRequest, ListParametersResponse>(
      name_ + "/list_parameters", [this](const ListParametersRequest& request) {
        ListParametersResponse response;
        const auto declared = context_.parameters().declared(name_);
        for (const auto& [name, value] : declared) {
          (void)value;
          bool matches = request.prefixes.empty();
          std::size_t depth = 0;
          for (const auto& prefix : request.prefixes) {
            if (prefix.empty() || name.compare(0, prefix.size(), prefix) == 0) {
              matches = true;
              std::size_t start = prefix.empty() ? 0 : prefix.size();
              if (start < name.size() && name[start] == '.') {
                ++start;
              }
              depth = 1;
              for (std::size_t i = start; i < name.size(); ++i) {
                if (name[i] == '.') {
                  ++depth;
                }
              }
              break;
            }
          }
          if (matches && (request.depth == 0 || depth <= request.depth)) {
            response.result.push_back(name);
          }
        }
        return response;
      });

  create_service<DescribeParametersRequest, DescribeParametersResponse>(
      name_ + "/describe_parameters", [this](const DescribeParametersRequest& request) {
        DescribeParametersResponse response;
        response.descriptors.reserve(request.names.size());
        for (const auto& name : request.names) {
          ParameterDescriptor descriptor;
          descriptor.name = name;
          try {
            descriptor.type = parameter_type_id(context_.parameters().get(name_, name));
          } catch (const std::exception&) {
          }
          response.descriptors.push_back(std::move(descriptor));
        }
        return response;
      });
}

Node::~Node() { shutdown(); }

void Node::shutdown() {
  for (auto& timer : timers_) {
    timer.cancel();
  }
  timers_.clear();
  for (auto& subscription : subscriptions_) {
    subscription->cancel();
  }
  subscriptions_.clear();
  for (auto& service : services_) {
    service.destroy();
  }
  services_.clear();
}

}  // namespace krosc
