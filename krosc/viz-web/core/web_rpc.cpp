#include "krosc/viz_web/web_rpc.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

#include "krosc/context.hpp"
#include "krosc/msgs.hpp"
#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/parameter_rpc.hpp"
#include "krosc/ros2_control/controller_manager.hpp"

namespace krosc::viz_web {

namespace {

Json parameter_value_json(const msgs::rcl_interfaces::ParameterValue& value) {
  using Type = msgs::rcl_interfaces::ParameterType;
  switch (static_cast<Type>(value.type)) {
    case Type::Bool:
      return Json(value.bool_value);
    case Type::Integer:
      return Json(value.integer_value);
    case Type::Double:
      return Json(value.double_value);
    case Type::String:
      return Json(value.string_value);
    case Type::BoolArray: {
      Json array = Json::array();
      for (const bool item : value.bool_array_value) {
        array.push(Json(item));
      }
      return array;
    }
    case Type::IntegerArray: {
      Json array = Json::array();
      for (const std::int64_t item : value.integer_array_value) {
        array.push(Json(item));
      }
      return array;
    }
    case Type::DoubleArray: {
      Json array = Json::array();
      for (const double item : value.double_array_value) {
        array.push(Json(item));
      }
      return array;
    }
    case Type::StringArray: {
      Json array = Json::array();
      for (const auto& item : value.string_array_value) {
        array.push(Json(item));
      }
      return array;
    }
    case Type::NotSet:
    default:
      return Json();
  }
}

msgs::rcl_interfaces::ParameterValue json_parameter_value(const Json& value,
                                                          std::uint8_t hint) {
  using Type = msgs::rcl_interfaces::ParameterType;
  msgs::rcl_interfaces::ParameterValue result;
  Type type = static_cast<Type>(hint);
  if (type == Type::NotSet) {
    if (value.is_bool()) {
      type = Type::Bool;
    } else if (value.is_number()) {
      type = std::floor(value.as_double()) == value.as_double() ? Type::Integer : Type::Double;
    } else if (value.is_string()) {
      type = Type::String;
    } else if (value.is_array()) {
      const auto& items = value.as_array();
      if (!items.empty() && items.front().is_bool()) {
        type = Type::BoolArray;
      } else if (!items.empty() && items.front().is_string()) {
        type = Type::StringArray;
      } else if (!items.empty() && items.front().is_number() &&
                 std::floor(items.front().as_double()) == items.front().as_double()) {
        type = Type::IntegerArray;
      } else {
        type = Type::DoubleArray;
      }
    }
  }
  result.type = static_cast<std::uint8_t>(type);
  switch (type) {
    case Type::Bool:
      result.bool_value = value.as_bool();
      break;
    case Type::Integer:
      result.integer_value = value.as_int();
      break;
    case Type::Double:
      result.double_value = value.as_double();
      break;
    case Type::String:
      result.string_value = value.as_string();
      break;
    case Type::BoolArray:
      for (const auto& item : value.as_array()) {
        result.bool_array_value.push_back(item.as_bool());
      }
      break;
    case Type::IntegerArray:
      for (const auto& item : value.as_array()) {
        result.integer_array_value.push_back(item.as_int());
      }
      break;
    case Type::DoubleArray:
      for (const auto& item : value.as_array()) {
        result.double_array_value.push_back(item.as_double());
      }
      break;
    case Type::StringArray:
      for (const auto& item : value.as_array()) {
        result.string_array_value.push_back(item.as_string());
      }
      break;
    case Type::NotSet:
    default:
      break;
  }
  return result;
}

std::map<std::string, Json> json_object_map(const Json& value) {
  std::map<std::string, Json> result;
  for (const auto& [key, item] : value.as_object()) {
    result[key] = item;
  }
  return result;
}

}  // namespace

struct WebRpc::Impl {
  explicit Impl(Context& context_value) : context(context_value) {}
  ~Impl() { shutdown(); }

  Context& context;
  std::mutex methods_mutex;
  std::map<std::string, Handler> methods;
  struct Task {
    Json id;
    std::string method;
    Json params;
    Reply reply;
  };
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<Task> queue;
  std::thread worker;
  std::atomic<bool> stopping{false};
  bool started = false;

  void start() {
    if (started) {
      return;
    }
    started = true;
    stopping.store(false);
    worker = std::thread([this] { loop(); });
  }

  void shutdown() {
    stopping.store(true);
    queue_cv.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
  }

  void loop() {
    while (true) {
      Task task;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [this] { return stopping.load() || !queue.empty(); });
        if (queue.empty()) {
          if (stopping.load()) {
            return;
          }
          continue;
        }
        task = std::move(queue.front());
        queue.pop_front();
      }
      if (stopping.load()) {
        return;
      }
      Handler handler;
      {
        std::lock_guard<std::mutex> lock(methods_mutex);
        const auto found = methods.find(task.method);
        if (found != methods.end()) {
          handler = found->second;
        }
      }
      Json response = Json::object();
      response["id"] = task.id;
      if (!handler) {
        response["ok"] = false;
        response["error"] = "unknown method: " + task.method;
      } else {
        try {
          const Notify notify = [reply = task.reply](const std::string& event,
                                                     const Json& params) {
            Json message = Json::object();
            message["event"] = event;
            message["params"] = params;
            reply(message.dump());
          };
          Json result = handler(task.params, notify);
          response["ok"] = true;
          response["result"] = std::move(result);
        } catch (const std::exception& error) {
          response["ok"] = false;
          response["error"] = error.what();
        }
      }
      if (task.reply) {
        task.reply(response.dump());
      }
    }
  }
};

WebRpc::WebRpc(Context& context) : impl_(std::make_unique<Impl>(context)) {}
WebRpc::~WebRpc() { shutdown(); }

void WebRpc::add(std::string method, Handler handler) {
  std::lock_guard<std::mutex> lock(impl_->methods_mutex);
  impl_->methods[std::move(method)] = std::move(handler);
}

void WebRpc::start() { impl_->start(); }
void WebRpc::shutdown() { impl_->shutdown(); }

void WebRpc::handle(const std::string& text, Reply reply) {
  if (text.size() > 1024u * 1024u) {
    Json response = Json::object();
    response["ok"] = false;
    response["error"] = "RPC request too large";
    if (reply) {
      reply(response.dump());
    }
    return;
  }
  const auto parsed = Json::parse(text);
  if (!parsed || !parsed->is_object()) {
    Json response = Json::object();
    response["ok"] = false;
    response["error"] = "invalid JSON";
    if (reply) {
      reply(response.dump());
    }
    return;
  }
  Impl::Task task;
  task.id = (*parsed)["id"];
  task.method = (*parsed)["method"].as_string();
  task.params = (*parsed)["params"];
  task.reply = std::move(reply);
  if (task.method.empty()) {
    Json response = Json::object();
    response["id"] = task.id;
    response["ok"] = false;
    response["error"] = "method가 없다";
    if (task.reply) {
      task.reply(response.dump());
    }
    return;
  }
  {
    std::lock_guard<std::mutex> lock(impl_->queue_mutex);
    if (impl_->queue.size() >= 256) {
      impl_->queue.pop_front();
    }
    impl_->queue.push_back(std::move(task));
  }
  impl_->queue_cv.notify_one();
}

void register_default_web_rpc(WebRpc& rpc, Context& context) {
  rpc.add("map/load", [&context](const Json& params, const WebRpc::Notify&) {
    const std::string name = params["name"].as_string();
    const auto response = context.services().call<msgs::nav2::LoadMapRequest,
                                                  msgs::nav2::LoadMapResponse>(
        "map_catalog/load_map", msgs::nav2::LoadMapRequest{name});
    Json result = Json::object();
    result["result"] = response.result;
    return result;
  });

  rpc.add("map/save", [&context](const Json& params, const WebRpc::Notify&) {
    msgs::nav2::SaveMapRequest request;
    request.map_url = params["name"].as_string();
    const auto response = context.services().call<msgs::nav2::SaveMapRequest,
                                                  msgs::nav2::SaveMapResponse>(
        "map_catalog/save_map", request);
    Json result = Json::object();
    result["result"] = response.result;
    return result;
  });

  rpc.add("map/delete", [&context](const Json& params, const WebRpc::Notify&) {
    const auto response = context.services().call<msgs::nav2::DeleteMapRequest,
                                                  msgs::nav2::DeleteMapResponse>(
        "map_catalog/delete_map",
        msgs::nav2::DeleteMapRequest{params["name"].as_string()});
    Json result = Json::object();
    result["result"] = response.result;
    result["message"] = response.message;
    return result;
  });

  rpc.add("map/unload", [&context](const Json&, const WebRpc::Notify&) {
    (void)context.services().call<msgs::Empty, msgs::Empty>("map_catalog/unload_map",
                                                            msgs::Empty{});
    return Json::object();
  });

  rpc.add("map/slam_mode", [&context](const Json& params, const WebRpc::Notify&) {
    const auto response = context.services().call<msgs::String, msgs::Bool>(
        "map_catalog/set_slam_mode", msgs::String{params["mode"].as_string()});
    Json result = Json::object();
    result["ok"] = response.data;
    return result;
  });

  rpc.add("params/get", [&context](const Json& params, const WebRpc::Notify&) {
    msgs::rcl_interfaces::GetParametersRequest request;
    for (const auto& name : params["names"].as_array()) {
      request.names.push_back(name.as_string());
    }
    const std::string node_name = params["node"].as_string();
    const auto response = context.services().call<
        msgs::rcl_interfaces::GetParametersRequest,
        msgs::rcl_interfaces::GetParametersResponse>(node_name + "/get_parameters", request);
    Json values = Json::object();
    for (std::size_t i = 0; i < request.names.size() && i < response.values.size(); ++i) {
      values[request.names[i]] = parameter_value_json(response.values[i]);
    }
    Json result = Json::object();
    result["values"] = std::move(values);
    return result;
  });

  rpc.add("params/set", [&context](const Json& params, const WebRpc::Notify&) {
    const std::string node_name = params["node"].as_string();
    const auto requested = json_object_map(params["parameters"]);
    // 현재 타입을 먼저 읽어 int/double 구분을 보존한다.
    msgs::rcl_interfaces::GetParametersRequest query;
    for (const auto& [name, value] : requested) {
      (void)value;
      query.names.push_back(name);
    }
    std::map<std::string, std::uint8_t> hints;
    try {
      const auto current = context.services().call<
          msgs::rcl_interfaces::GetParametersRequest,
          msgs::rcl_interfaces::GetParametersResponse>(node_name + "/get_parameters", query);
      for (std::size_t i = 0; i < query.names.size() && i < current.values.size(); ++i) {
        hints[query.names[i]] = current.values[i].type;
      }
    } catch (const std::exception&) {
    }
    msgs::rcl_interfaces::SetParametersRequest request;
    for (const auto& [name, value] : requested) {
      msgs::rcl_interfaces::Parameter parameter;
      parameter.name = name;
      parameter.value = json_parameter_value(value, hints.count(name) ? hints[name] : 0);
      request.parameters.push_back(std::move(parameter));
    }
    const auto response = context.services().call<
        msgs::rcl_interfaces::SetParametersRequest,
        msgs::rcl_interfaces::SetParametersResponse>(node_name + "/set_parameters", request);
    Json results = Json::array();
    for (const auto& item : response.results) {
      Json entry = Json::object();
      entry["successful"] = item.successful;
      entry["reason"] = item.reason;
      results.push(std::move(entry));
    }
    Json result = Json::object();
    result["results"] = std::move(results);
    return result;
  });

  rpc.add("controller/switch", [&context](const Json& params, const WebRpc::Notify&) {
    ros2_control::SwitchControllerRequest request;
    for (const auto& item : params["start"].as_array()) {
      request.start_controllers.push_back(item.as_string());
    }
    for (const auto& item : params["stop"].as_array()) {
      request.stop_controllers.push_back(item.as_string());
    }
    request.strictness = params["strictness"].as_int(1);
    const auto response = context.services().call<ros2_control::SwitchControllerRequest,
                                                  ros2_control::SwitchControllerResponse>(
        "controller_manager/switch_controller", request);
    Json result = Json::object();
    result["ok"] = response.ok;
    result["message"] = response.message;
    return result;
  });

  rpc.add("rl/reload", [&context](const Json&, const WebRpc::Notify&) {
    const auto response = context.services().call<msgs::TriggerRequest, msgs::TriggerResponse>(
        "rl_nav_controller/reload", msgs::TriggerRequest{});
    Json result = Json::object();
    result["ok"] = response.success;
    result["message"] = response.message;
    return result;
  });
}

}  // namespace krosc::viz_web
