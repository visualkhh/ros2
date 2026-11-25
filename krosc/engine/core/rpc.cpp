#include "krosc/rpc.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <typeinfo>
#include <utility>

#include "krosc/msgs.hpp"
#include "krosc/msgs/control_msgs.hpp"
#include "krosc/msgs/moveit_msgs.hpp"
#include "krosc/msgs/nav2_srv.hpp"
#include "krosc/msgs/rcl_interfaces.hpp"
#include "krosc/ros2_control/controller_manager.hpp"

#include "net_codec.hpp"

namespace krosc::net {

namespace {

using detail::Reader;
using detail::Writer;

void encode_header(Writer& w, const msgs::Header& header) {
  w.f64(header.stamp);
  w.str(header.frame_id);
}
void encode_quaternion(Writer& w, const msgs::Quaternion& value) {
  w.f64(value.x);
  w.f64(value.y);
  w.f64(value.z);
  w.f64(value.w);
}
void encode_point(Writer& w, const msgs::Point& value) {
  w.f64(value.x);
  w.f64(value.y);
  w.f64(value.z);
}
void encode_pose(Writer& w, const msgs::Pose& value) {
  encode_point(w, value.position);
  encode_quaternion(w, value.orientation);
}
void encode_pose_stamped(Writer& w, const msgs::PoseStamped& value) {
  encode_header(w, value.header);
  encode_pose(w, value.pose);
}
void encode_doubles(Writer& w, const std::vector<double>& values) {
  w.u32(static_cast<std::uint32_t>(values.size()));
  for (const double value : values) {
    w.f64(value);
  }
}
void encode_strings(Writer& w, const std::vector<std::string>& values) {
  w.u32(static_cast<std::uint32_t>(values.size()));
  for (const auto& value : values) {
    w.str(value);
  }
}

msgs::Header decode_header(Reader& r) {
  msgs::Header header;
  header.stamp = r.f64();
  header.frame_id = r.str();
  return header;
}
msgs::Quaternion decode_quaternion(Reader& r) { return {r.f64(), r.f64(), r.f64(), r.f64()}; }
msgs::Point decode_point(Reader& r) { return {r.f64(), r.f64(), r.f64()}; }
msgs::Pose decode_pose(Reader& r) {
  msgs::Pose pose;
  pose.position = decode_point(r);
  pose.orientation = decode_quaternion(r);
  return pose;
}
msgs::PoseStamped decode_pose_stamped(Reader& r) {
  msgs::PoseStamped pose;
  pose.header = decode_header(r);
  pose.pose = decode_pose(r);
  return pose;
}
std::vector<double> decode_doubles(Reader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<double> values;
  values.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    values.push_back(r.f64());
  }
  return values;
}
std::vector<std::string> decode_strings(Reader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<std::string> values;
  values.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    values.push_back(r.str());
  }
  return values;
}

void encode_joint_trajectory_point(Writer& w, const msgs::control::JointTrajectoryPoint& point) {
  encode_doubles(w, point.positions);
  encode_doubles(w, point.velocities);
  encode_doubles(w, point.accelerations);
  encode_doubles(w, point.effort);
  w.f64(point.time_from_start);
}

msgs::control::JointTrajectoryPoint decode_joint_trajectory_point(Reader& r) {
  msgs::control::JointTrajectoryPoint point;
  point.positions = decode_doubles(r);
  point.velocities = decode_doubles(r);
  point.accelerations = decode_doubles(r);
  point.effort = decode_doubles(r);
  point.time_from_start = r.f64();
  return point;
}

void encode_joint_trajectory(Writer& w, const msgs::control::JointTrajectory& trajectory) {
  encode_header(w, trajectory.header);
  encode_strings(w, trajectory.joint_names);
  w.u32(static_cast<std::uint32_t>(trajectory.points.size()));
  for (const auto& point : trajectory.points) {
    encode_joint_trajectory_point(w, point);
  }
}

msgs::control::JointTrajectory decode_joint_trajectory(Reader& r) {
  msgs::control::JointTrajectory trajectory;
  trajectory.header = decode_header(r);
  trajectory.joint_names = decode_strings(r);
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return trajectory;
  }
  trajectory.points.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    trajectory.points.push_back(decode_joint_trajectory_point(r));
  }
  return trajectory;
}

void encode_tolerances(Writer& w, const std::vector<msgs::control::JointTolerance>& values) {
  w.u32(static_cast<std::uint32_t>(values.size()));
  for (const auto& value : values) {
    w.str(value.name);
    w.f64(value.position);
    w.f64(value.velocity);
    w.f64(value.acceleration);
  }
}

std::vector<msgs::control::JointTolerance> decode_tolerances(Reader& r) {
  const std::uint32_t count = r.u32();
  if (count > 1000000) {
    r.skip(r.remaining());
    return {};
  }
  std::vector<msgs::control::JointTolerance> values;
  values.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    msgs::control::JointTolerance value;
    value.name = r.str();
    value.position = r.f64();
    value.velocity = r.f64();
    value.acceleration = r.f64();
    values.push_back(std::move(value));
  }
  return values;
}

void encode_parameter_value(Writer& w, const msgs::rcl_interfaces::ParameterValue& value) {
  w.u8(value.type);
  using Type = msgs::rcl_interfaces::ParameterType;
  switch (static_cast<Type>(value.type)) {
    case Type::Bool:
      w.u8(value.bool_value ? 1U : 0U);
      break;
    case Type::Integer:
      w.i64(value.integer_value);
      break;
    case Type::Double:
      w.f64(value.double_value);
      break;
    case Type::String:
      w.str(value.string_value);
      break;
    case Type::BoolArray:
      w.u32(static_cast<std::uint32_t>(value.bool_array_value.size()));
      for (const bool item : value.bool_array_value) {
        w.u8(item ? 1U : 0U);
      }
      break;
    case Type::IntegerArray:
      w.u32(static_cast<std::uint32_t>(value.integer_array_value.size()));
      for (const std::int64_t item : value.integer_array_value) {
        w.i64(item);
      }
      break;
    case Type::DoubleArray:
      encode_doubles(w, value.double_array_value);
      break;
    case Type::StringArray:
      encode_strings(w, value.string_array_value);
      break;
    case Type::NotSet:
    default:
      break;
  }
}

msgs::rcl_interfaces::ParameterValue decode_parameter_value(Reader& r) {
  using Type = msgs::rcl_interfaces::ParameterType;
  msgs::rcl_interfaces::ParameterValue value;
  value.type = r.u8();
  switch (static_cast<Type>(value.type)) {
    case Type::Bool:
      value.bool_value = r.u8() != 0;
      break;
    case Type::Integer:
      value.integer_value = r.i64();
      break;
    case Type::Double:
      value.double_value = r.f64();
      break;
    case Type::String:
      value.string_value = r.str();
      break;
    case Type::BoolArray: {
      const std::uint32_t count = r.u32();
      if (count > 1000000) {
        r.skip(r.remaining());
        break;
      }
      value.bool_array_value.reserve(count);
      for (std::uint32_t i = 0; i < count; ++i) {
        value.bool_array_value.push_back(r.u8() != 0);
      }
      break;
    }
    case Type::IntegerArray: {
      const std::uint32_t count = r.u32();
      if (count > 1000000) {
        r.skip(r.remaining());
        break;
      }
      value.integer_array_value.reserve(count);
      for (std::uint32_t i = 0; i < count; ++i) {
        value.integer_array_value.push_back(r.i64());
      }
      break;
    }
    case Type::DoubleArray:
      value.double_array_value = decode_doubles(r);
      break;
    case Type::StringArray:
      value.string_array_value = decode_strings(r);
      break;
    case Type::NotSet:
    default:
      break;
  }
  return value;
}

struct Codec {
  std::string name;
  std::function<std::vector<std::uint8_t>(const void*)> encode;
  std::function<std::any(Reader&)> decode;
};

struct CodecTable {
  std::map<std::type_index, Codec> by_type;
  std::map<std::string, std::type_index> by_name;
};

template <class T>
void add_codec(CodecTable& table, std::string name,
               std::function<void(Writer&, const T&)> encoder,
               std::function<T(Reader&)> decoder) {
  Codec codec;
  codec.name = name;
  codec.encode = [encoder = std::move(encoder)](const void* value) {
    Writer writer;
    encoder(writer, *static_cast<const T*>(value));
    return std::move(writer.data);
  };
  codec.decode = [decoder = std::move(decoder)](Reader& reader) -> std::any {
    T value = decoder(reader);
    if (!reader.ok()) {
      return {};
    }
    return std::any(std::move(value));
  };
  table.by_name.emplace(name, std::type_index(typeid(T)));
  table.by_type[std::type_index(typeid(T))] = std::move(codec);
}

const CodecTable& codec_table() {
  static const CodecTable table = [] {
    CodecTable table;
    add_codec<msgs::Empty>(table, "std_msgs/Empty", [](Writer&, const msgs::Empty&) {},
                           [](Reader&) { return msgs::Empty{}; });
    add_codec<msgs::String>(table, "std_msgs/String",
                            [](Writer& w, const msgs::String& value) { w.str(value.data); },
                            [](Reader& r) { return msgs::String{r.str()}; });
    add_codec<msgs::Bool>(table, "std_msgs/Bool",
                          [](Writer& w, const msgs::Bool& value) { w.u8(value.data ? 1U : 0U); },
                          [](Reader& r) { return msgs::Bool{r.u8() != 0}; });

    add_codec<msgs::SetBoolRequest>(table, "std_srvs/SetBool_Request",
                                    [](Writer& w, const msgs::SetBoolRequest& value) {
                                      w.u8(value.data ? 1U : 0U);
                                    },
                                    [](Reader& r) { return msgs::SetBoolRequest{r.u8() != 0}; });
    add_codec<msgs::SetBoolResponse>(
        table, "std_srvs/SetBool_Response",
        [](Writer& w, const msgs::SetBoolResponse& value) {
          w.u8(value.success ? 1U : 0U);
          w.str(value.message);
        },
        [](Reader& r) {
          msgs::SetBoolResponse value;
          value.success = r.u8() != 0;
          value.message = r.str();
          return value;
        });
    add_codec<msgs::TriggerRequest>(table, "std_srvs/Trigger_Request",
                                    [](Writer&, const msgs::TriggerRequest&) {},
                                    [](Reader&) { return msgs::TriggerRequest{}; });
    add_codec<msgs::TriggerResponse>(
        table, "std_srvs/Trigger_Response",
        [](Writer& w, const msgs::TriggerResponse& value) {
          w.u8(value.success ? 1U : 0U);
          w.str(value.message);
        },
        [](Reader& r) {
          msgs::TriggerResponse value;
          value.success = r.u8() != 0;
          value.message = r.str();
          return value;
        });

    add_codec<msgs::nav2::LoadMapRequest>(
        table, "nav2_msgs/srv/LoadMap_Request",
        [](Writer& w, const msgs::nav2::LoadMapRequest& value) { w.str(value.map_url); },
        [](Reader& r) { return msgs::nav2::LoadMapRequest{r.str()}; });
    add_codec<msgs::nav2::LoadMapResponse>(
        table, "nav2_msgs/srv/LoadMap_Response",
        [](Writer& w, const msgs::nav2::LoadMapResponse& value) { w.u8(static_cast<std::uint8_t>(value.result)); },
        [](Reader& r) {
          msgs::nav2::LoadMapResponse value;
          value.result = r.u8();
          return value;
        });
    add_codec<msgs::nav2::SaveMapRequest>(
        table, "nav2_msgs/srv/SaveMap_Request",
        [](Writer& w, const msgs::nav2::SaveMapRequest& value) {
          w.str(value.map_topic);
          w.str(value.map_url);
          w.str(value.image_format);
          w.str(value.map_mode);
          w.f64(value.free_thresh);
          w.f64(value.occupied_thresh);
        },
        [](Reader& r) {
          msgs::nav2::SaveMapRequest value;
          value.map_topic = r.str();
          value.map_url = r.str();
          value.image_format = r.str();
          value.map_mode = r.str();
          value.free_thresh = r.f64();
          value.occupied_thresh = r.f64();
          return value;
        });
    add_codec<msgs::nav2::SaveMapResponse>(
        table, "nav2_msgs/srv/SaveMap_Response",
        [](Writer& w, const msgs::nav2::SaveMapResponse& value) { w.u8(value.result ? 1U : 0U); },
        [](Reader& r) { return msgs::nav2::SaveMapResponse{r.u8() != 0}; });
    add_codec<msgs::nav2::DeleteMapRequest>(
        table, "nav2_msgs/srv/DeleteMap_Request",
        [](Writer& w, const msgs::nav2::DeleteMapRequest& value) { w.str(value.map_url); },
        [](Reader& r) { return msgs::nav2::DeleteMapRequest{r.str()}; });
    add_codec<msgs::nav2::DeleteMapResponse>(
        table, "nav2_msgs/srv/DeleteMap_Response",
        [](Writer& w, const msgs::nav2::DeleteMapResponse& value) {
          w.u8(value.result ? 1U : 0U);
          w.str(value.message);
        },
        [](Reader& r) {
          msgs::nav2::DeleteMapResponse value;
          value.result = r.u8() != 0;
          value.message = r.str();
          return value;
        });

    add_codec<msgs::rcl_interfaces::GetParametersRequest>(
        table, "rcl_interfaces/srv/GetParameters_Request",
        [](Writer& w, const msgs::rcl_interfaces::GetParametersRequest& value) {
          encode_strings(w, value.names);
        },
        [](Reader& r) {
          msgs::rcl_interfaces::GetParametersRequest value;
          value.names = decode_strings(r);
          return value;
        });
    add_codec<msgs::rcl_interfaces::GetParametersResponse>(
        table, "rcl_interfaces/srv/GetParameters_Response",
        [](Writer& w, const msgs::rcl_interfaces::GetParametersResponse& value) {
          w.u32(static_cast<std::uint32_t>(value.values.size()));
          for (const auto& item : value.values) {
            encode_parameter_value(w, item);
          }
        },
        [](Reader& r) {
          msgs::rcl_interfaces::GetParametersResponse value;
          const std::uint32_t count = r.u32();
          if (count > 1000000) {
            r.skip(r.remaining());
            return value;
          }
          value.values.reserve(count);
          for (std::uint32_t i = 0; i < count; ++i) {
            value.values.push_back(decode_parameter_value(r));
          }
          return value;
        });
    add_codec<msgs::rcl_interfaces::SetParametersRequest>(
        table, "rcl_interfaces/srv/SetParameters_Request",
        [](Writer& w, const msgs::rcl_interfaces::SetParametersRequest& value) {
          w.u32(static_cast<std::uint32_t>(value.parameters.size()));
          for (const auto& parameter : value.parameters) {
            w.str(parameter.name);
            encode_parameter_value(w, parameter.value);
          }
        },
        [](Reader& r) {
          msgs::rcl_interfaces::SetParametersRequest value;
          const std::uint32_t count = r.u32();
          if (count > 1000000) {
            r.skip(r.remaining());
            return value;
          }
          value.parameters.reserve(count);
          for (std::uint32_t i = 0; i < count; ++i) {
            msgs::rcl_interfaces::Parameter parameter;
            parameter.name = r.str();
            parameter.value = decode_parameter_value(r);
            value.parameters.push_back(std::move(parameter));
          }
          return value;
        });
    add_codec<msgs::rcl_interfaces::SetParametersResponse>(
        table, "rcl_interfaces/srv/SetParameters_Response",
        [](Writer& w, const msgs::rcl_interfaces::SetParametersResponse& value) {
          w.u32(static_cast<std::uint32_t>(value.results.size()));
          for (const auto& item : value.results) {
            w.u8(item.successful ? 1U : 0U);
            w.str(item.reason);
          }
        },
        [](Reader& r) {
          msgs::rcl_interfaces::SetParametersResponse value;
          const std::uint32_t count = r.u32();
          if (count > 1000000) {
            r.skip(r.remaining());
            return value;
          }
          value.results.reserve(count);
          for (std::uint32_t i = 0; i < count; ++i) {
            msgs::rcl_interfaces::SetParametersResult item;
            item.successful = r.u8() != 0;
            item.reason = r.str();
            value.results.push_back(std::move(item));
          }
          return value;
        });
    add_codec<msgs::rcl_interfaces::ListParametersRequest>(
        table, "rcl_interfaces/srv/ListParameters_Request",
        [](Writer& w, const msgs::rcl_interfaces::ListParametersRequest& value) {
          encode_strings(w, value.prefixes);
          w.u64(value.depth);
        },
        [](Reader& r) {
          msgs::rcl_interfaces::ListParametersRequest value;
          value.prefixes = decode_strings(r);
          value.depth = r.u64();
          return value;
        });
    add_codec<msgs::rcl_interfaces::ListParametersResponse>(
        table, "rcl_interfaces/srv/ListParameters_Response",
        [](Writer& w, const msgs::rcl_interfaces::ListParametersResponse& value) {
          encode_strings(w, value.result);
        },
        [](Reader& r) {
          msgs::rcl_interfaces::ListParametersResponse value;
          value.result = decode_strings(r);
          return value;
        });
    add_codec<msgs::rcl_interfaces::DescribeParametersRequest>(
        table, "rcl_interfaces/srv/DescribeParameters_Request",
        [](Writer& w, const msgs::rcl_interfaces::DescribeParametersRequest& value) {
          encode_strings(w, value.names);
        },
        [](Reader& r) {
          msgs::rcl_interfaces::DescribeParametersRequest value;
          value.names = decode_strings(r);
          return value;
        });
    add_codec<msgs::rcl_interfaces::DescribeParametersResponse>(
        table, "rcl_interfaces/srv/DescribeParameters_Response",
        [](Writer& w, const msgs::rcl_interfaces::DescribeParametersResponse& value) {
          w.u32(static_cast<std::uint32_t>(value.descriptors.size()));
          for (const auto& descriptor : value.descriptors) {
            w.str(descriptor.name);
            w.u8(descriptor.type);
            w.str(descriptor.description);
            w.str(descriptor.additional_info);
            w.u8(descriptor.read_only ? 1U : 0U);
          }
        },
        [](Reader& r) {
          msgs::rcl_interfaces::DescribeParametersResponse value;
          const std::uint32_t count = r.u32();
          if (count > 1000000) {
            r.skip(r.remaining());
            return value;
          }
          value.descriptors.reserve(count);
          for (std::uint32_t i = 0; i < count; ++i) {
            msgs::rcl_interfaces::ParameterDescriptor descriptor;
            descriptor.name = r.str();
            descriptor.type = r.u8();
            descriptor.description = r.str();
            descriptor.additional_info = r.str();
            descriptor.read_only = r.u8() != 0;
            value.descriptors.push_back(std::move(descriptor));
          }
          return value;
        });

    add_codec<ros2_control::ListControllersRequest>(
        table, "controller_manager_msgs/srv/ListControllers_Request",
        [](Writer&, const ros2_control::ListControllersRequest&) {},
        [](Reader&) { return ros2_control::ListControllersRequest{}; });
    add_codec<ros2_control::ListControllersResponse>(
        table, "controller_manager_msgs/srv/ListControllers_Response",
        [](Writer& w, const ros2_control::ListControllersResponse& value) {
          encode_strings(w, value.name);
          encode_strings(w, value.type);
          encode_strings(w, value.state);
        },
        [](Reader& r) {
          ros2_control::ListControllersResponse value;
          value.name = decode_strings(r);
          value.type = decode_strings(r);
          value.state = decode_strings(r);
          return value;
        });
    add_codec<ros2_control::LoadControllerRequest>(
        table, "controller_manager_msgs/srv/LoadController_Request",
        [](Writer& w, const ros2_control::LoadControllerRequest& value) { w.str(value.name); },
        [](Reader& r) { return ros2_control::LoadControllerRequest{r.str()}; });
    add_codec<ros2_control::LoadControllerResponse>(
        table, "controller_manager_msgs/srv/LoadController_Response",
        [](Writer& w, const ros2_control::LoadControllerResponse& value) {
          w.u8(value.ok ? 1U : 0U);
          w.str(value.message);
        },
        [](Reader& r) {
          ros2_control::LoadControllerResponse value;
          value.ok = r.u8() != 0;
          value.message = r.str();
          return value;
        });
    add_codec<ros2_control::ConfigureControllerRequest>(
        table, "controller_manager_msgs/srv/ConfigureController_Request",
        [](Writer& w, const ros2_control::ConfigureControllerRequest& value) { w.str(value.name); },
        [](Reader& r) { return ros2_control::ConfigureControllerRequest{r.str()}; });
    add_codec<ros2_control::ConfigureControllerResponse>(
        table, "controller_manager_msgs/srv/ConfigureController_Response",
        [](Writer& w, const ros2_control::ConfigureControllerResponse& value) {
          w.u8(value.ok ? 1U : 0U);
          w.str(value.message);
        },
        [](Reader& r) {
          ros2_control::ConfigureControllerResponse value;
          value.ok = r.u8() != 0;
          value.message = r.str();
          return value;
        });
    add_codec<ros2_control::SwitchControllerRequest>(
        table, "controller_manager_msgs/srv/SwitchController_Request",
        [](Writer& w, const ros2_control::SwitchControllerRequest& value) {
          encode_strings(w, value.start_controllers);
          encode_strings(w, value.stop_controllers);
          w.i64(value.strictness);
          w.u8(value.start_asap ? 1U : 0U);
          w.f64(value.timeout);
        },
        [](Reader& r) {
          ros2_control::SwitchControllerRequest value;
          value.start_controllers = decode_strings(r);
          value.stop_controllers = decode_strings(r);
          value.strictness = r.i64();
          value.start_asap = r.u8() != 0;
          value.timeout = r.f64();
          return value;
        });
    add_codec<ros2_control::SwitchControllerResponse>(
        table, "controller_manager_msgs/srv/SwitchController_Response",
        [](Writer& w, const ros2_control::SwitchControllerResponse& value) {
          w.u8(value.ok ? 1U : 0U);
          w.str(value.message);
        },
        [](Reader& r) {
          ros2_control::SwitchControllerResponse value;
          value.ok = r.u8() != 0;
          value.message = r.str();
          return value;
        });

    // 액션 응답(서비스 본문). Goal 요청은 action codec으로 따로 인코딩한다.
    add_codec<SendGoalResponse>(
        table, "krosc/action/SendGoalResponse",
        [](Writer& w, const SendGoalResponse& value) { w.u8(value.accepted ? 1U : 0U); },
        [](Reader& r) { return SendGoalResponse{r.u8() != 0}; });
    add_codec<CancelGoalResponse>(
        table, "krosc/action/CancelGoalResponse",
        [](Writer& w, const CancelGoalResponse& value) { w.u8(value.canceled ? 1U : 0U); },
        [](Reader& r) { return CancelGoalResponse{r.u8() != 0}; });
    return table;
  }();
  return table;
}

std::mutex& action_codec_mutex() {
  static std::mutex mutex;
  return mutex;
}

std::map<std::string, ActionCodec>& action_codecs() {
  static std::map<std::string, ActionCodec> codecs;
  return codecs;
}

bool matches_action(const std::string& name, const std::string& codec_name) {
  if (name == codec_name) {
    return true;
  }
  if (name.size() <= codec_name.size()) {
    return false;
  }
  const std::size_t offset = name.size() - codec_name.size();
  return name[offset - 1] == '/' && name.compare(offset, codec_name.size(), codec_name) == 0;
}

const ActionCodec* find_action_codec_locked(const std::string& action_name) {
  for (const auto& [name, codec] : action_codecs()) {
    (void)name;
    if (matches_action(action_name, codec.name)) {
      return &codec;
    }
  }
  return nullptr;
}

void add_action_codec_locked(ActionCodec codec) {
  action_codecs()[codec.name] = std::move(codec);
}

void ensure_builtin_action_codecs() {
  static std::once_flag once;
  std::call_once(once, [] {
    ActionCodec move_group;
    move_group.name = "move_action";
    move_group.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
      const auto* goal = std::any_cast<msgs::moveit::MoveGroupGoal>(&value);
      if (goal == nullptr) {
        return {};
      }
      Writer w;
      encode_pose_stamped(w, goal->target_pose);
      encode_doubles(w, goal->joint_goal);
      w.str(goal->planner_id);
      w.str(goal->controller);
      w.u8(goal->cartesian ? 1U : 0U);
      w.u8(goal->plan_only ? 1U : 0U);
      w.f64(goal->grip);
      w.str(goal->preset);
      return std::move(w.data);
    };
    move_group.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
      Reader r(data, size);
      msgs::moveit::MoveGroupGoal goal;
      goal.target_pose = decode_pose_stamped(r);
      goal.joint_goal = decode_doubles(r);
      goal.planner_id = r.str();
      goal.controller = r.str();
      goal.cartesian = r.u8() != 0;
      goal.plan_only = r.u8() != 0;
      goal.grip = r.f64();
      goal.preset = r.str();
      if (!r.ok()) {
        return {};
      }
      return std::any(std::move(goal));
    };
    move_group.encode_feedback = [](const std::any& value) -> std::vector<std::uint8_t> {
      const auto* feedback = std::any_cast<msgs::moveit::MoveGroupFeedback>(&value);
      if (feedback == nullptr) {
        return {};
      }
      Writer w;
      w.f64(feedback->progress);
      w.str(feedback->state);
      return std::move(w.data);
    };
    move_group.decode_feedback = [](const std::uint8_t* data, std::size_t size) -> std::any {
      Reader r(data, size);
      msgs::moveit::MoveGroupFeedback feedback;
      feedback.progress = r.f64();
      feedback.state = r.str();
      if (!r.ok()) {
        return {};
      }
      return std::any(std::move(feedback));
    };
    move_group.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
      const auto* result = std::any_cast<msgs::moveit::MoveGroupResult>(&value);
      if (result == nullptr) {
        return {};
      }
      Writer w;
      w.u32(static_cast<std::uint32_t>(result->error_code));
      w.str(result->error_msg);
      encode_joint_trajectory(w, result->trajectory);
      return std::move(w.data);
    };
    move_group.decode_result = [](const std::uint8_t* data, std::size_t size) -> std::any {
      Reader r(data, size);
      msgs::moveit::MoveGroupResult result;
      result.error_code = static_cast<int>(r.u32());
      result.error_msg = r.str();
      result.trajectory = decode_joint_trajectory(r);
      if (!r.ok()) {
        return {};
      }
      return std::any(std::move(result));
    };
    add_action_codec_locked(std::move(move_group));

    ActionCodec follow;
    follow.name = "follow_joint_trajectory";
    follow.encode_goal = [](const std::any& value) -> std::vector<std::uint8_t> {
      const auto* goal = std::any_cast<msgs::control::FollowJointTrajectoryGoal>(&value);
      if (goal == nullptr) {
        return {};
      }
      Writer w;
      encode_joint_trajectory(w, goal->trajectory);
      encode_tolerances(w, goal->path_tolerance);
      encode_tolerances(w, goal->goal_tolerance);
      w.f64(goal->goal_time_tolerance);
      return std::move(w.data);
    };
    follow.decode_goal = [](const std::uint8_t* data, std::size_t size) -> std::any {
      Reader r(data, size);
      msgs::control::FollowJointTrajectoryGoal goal;
      goal.trajectory = decode_joint_trajectory(r);
      goal.path_tolerance = decode_tolerances(r);
      goal.goal_tolerance = decode_tolerances(r);
      goal.goal_time_tolerance = r.f64();
      if (!r.ok()) {
        return {};
      }
      return std::any(std::move(goal));
    };
    follow.encode_feedback = [](const std::any& value) -> std::vector<std::uint8_t> {
      const auto* feedback = std::any_cast<msgs::control::FollowJointTrajectoryFeedback>(&value);
      if (feedback == nullptr) {
        return {};
      }
      Writer w;
      encode_header(w, feedback->header);
      encode_strings(w, feedback->joint_names);
      encode_joint_trajectory_point(w, feedback->desired);
      encode_joint_trajectory_point(w, feedback->actual);
      encode_joint_trajectory_point(w, feedback->error);
      return std::move(w.data);
    };
    follow.decode_feedback = [](const std::uint8_t* data, std::size_t size) -> std::any {
      Reader r(data, size);
      msgs::control::FollowJointTrajectoryFeedback feedback;
      feedback.header = decode_header(r);
      feedback.joint_names = decode_strings(r);
      feedback.desired = decode_joint_trajectory_point(r);
      feedback.actual = decode_joint_trajectory_point(r);
      feedback.error = decode_joint_trajectory_point(r);
      if (!r.ok()) {
        return {};
      }
      return std::any(std::move(feedback));
    };
    follow.encode_result = [](const std::any& value) -> std::vector<std::uint8_t> {
      const auto* result = std::any_cast<msgs::control::FollowJointTrajectoryResult>(&value);
      if (result == nullptr) {
        return {};
      }
      Writer w;
      w.u32(static_cast<std::uint32_t>(result->error_code));
      w.str(result->error_string);
      return std::move(w.data);
    };
    follow.decode_result = [](const std::uint8_t* data, std::size_t size) -> std::any {
      Reader r(data, size);
      msgs::control::FollowJointTrajectoryResult result;
      result.error_code = static_cast<int>(r.u32());
      result.error_string = r.str();
      if (!r.ok()) {
        return {};
      }
      return std::any(std::move(result));
    };
    add_action_codec_locked(std::move(follow));
  });
}

}  // namespace

std::vector<std::uint8_t> encode_rpc_envelope(const RpcEnvelope& envelope) {
  Writer writer;
  writer.u32(envelope.request_id);
  writer.u8(envelope.status);
  writer.str(envelope.request_type);
  writer.str(envelope.response_type);
  writer.u32(static_cast<std::uint32_t>(envelope.payload.size()));
  writer.bytes(envelope.payload);
  writer.str(envelope.error);
  return std::move(writer.data);
}

std::optional<RpcEnvelope> decode_rpc_envelope(const std::vector<std::uint8_t>& payload) {
  Reader reader(payload);
  RpcEnvelope envelope;
  envelope.request_id = reader.u32();
  envelope.status = reader.u8();
  envelope.request_type = reader.str();
  envelope.response_type = reader.str();
  const std::uint32_t length = reader.u32();
  if (!reader.ok() || length > reader.remaining()) {
    return std::nullopt;
  }
  envelope.payload.assign(reader.rest(), reader.rest() + length);
  reader.skip(length);
  envelope.error = reader.str();
  if (!reader.ok()) {
    return std::nullopt;
  }
  return envelope;
}

std::optional<std::string> rpc_type_name(std::type_index type) {
  const auto& table = codec_table();
  const auto found = table.by_type.find(type);
  if (found == table.by_type.end()) {
    return std::nullopt;
  }
  return found->second.name;
}

std::vector<std::uint8_t> encode_rpc_value(std::type_index type, const void* value) {
  const auto& table = codec_table();
  const auto found = table.by_type.find(type);
  if (found == table.by_type.end() || value == nullptr) {
    return {};
  }
  return found->second.encode(value);
}

bool rpc_type_registered(std::type_index type) {
  return codec_table().by_type.count(type) != 0;
}

namespace {

template <class T>
bool try_encode_any(std::type_index type, const std::any& value, std::vector<std::uint8_t>& out) {
  if (type != std::type_index(typeid(T))) {
    return false;
  }
  const T* pointer = std::any_cast<T>(&value);
  if (pointer == nullptr) {
    return false;
  }
  out = encode_rpc_value(type, pointer);
  return true;
}

}  // namespace

std::vector<std::uint8_t> encode_rpc_any(std::type_index type, const std::any& value) {
  std::vector<std::uint8_t> out;
  if (try_encode_any<msgs::Empty>(type, value, out) ||
      try_encode_any<msgs::String>(type, value, out) ||
      try_encode_any<msgs::Bool>(type, value, out) ||
      try_encode_any<msgs::SetBoolRequest>(type, value, out) ||
      try_encode_any<msgs::SetBoolResponse>(type, value, out) ||
      try_encode_any<msgs::TriggerRequest>(type, value, out) ||
      try_encode_any<msgs::TriggerResponse>(type, value, out) ||
      try_encode_any<msgs::nav2::LoadMapRequest>(type, value, out) ||
      try_encode_any<msgs::nav2::LoadMapResponse>(type, value, out) ||
      try_encode_any<msgs::nav2::SaveMapRequest>(type, value, out) ||
      try_encode_any<msgs::nav2::SaveMapResponse>(type, value, out) ||
      try_encode_any<msgs::nav2::DeleteMapRequest>(type, value, out) ||
      try_encode_any<msgs::nav2::DeleteMapResponse>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::GetParametersRequest>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::GetParametersResponse>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::SetParametersRequest>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::SetParametersResponse>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::ListParametersRequest>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::ListParametersResponse>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::DescribeParametersRequest>(type, value, out) ||
      try_encode_any<msgs::rcl_interfaces::DescribeParametersResponse>(type, value, out) ||
      try_encode_any<ros2_control::ListControllersRequest>(type, value, out) ||
      try_encode_any<ros2_control::ListControllersResponse>(type, value, out) ||
      try_encode_any<ros2_control::LoadControllerRequest>(type, value, out) ||
      try_encode_any<ros2_control::LoadControllerResponse>(type, value, out) ||
      try_encode_any<ros2_control::ConfigureControllerRequest>(type, value, out) ||
      try_encode_any<ros2_control::ConfigureControllerResponse>(type, value, out) ||
      try_encode_any<ros2_control::SwitchControllerRequest>(type, value, out) ||
      try_encode_any<ros2_control::SwitchControllerResponse>(type, value, out) ||
      try_encode_any<SendGoalResponse>(type, value, out) ||
      try_encode_any<CancelGoalResponse>(type, value, out)) {
    return out;
  }
  return {};
}

std::any decode_rpc_value(const std::string& type_name, const std::uint8_t* data,
                          std::size_t size) {
  const auto& table = codec_table();
  const auto name_it = table.by_name.find(type_name);
  if (name_it == table.by_name.end()) {
    return {};
  }
  const auto type_it = table.by_type.find(name_it->second);
  if (type_it == table.by_type.end()) {
    return {};
  }
  Reader reader(data, size);
  return type_it->second.decode(reader);
}

void register_action_codec(ActionCodec codec) {
  ensure_builtin_action_codecs();
  std::lock_guard<std::mutex> lock(action_codec_mutex());
  add_action_codec_locked(std::move(codec));
}

const ActionCodec* find_action_codec(const std::string& action_name) {
  ensure_builtin_action_codecs();
  std::lock_guard<std::mutex> lock(action_codec_mutex());
  return find_action_codec_locked(action_name);
}

std::string action_name_from_service(const std::string& service) {
  for (const char* suffix : {"/send_goal", "/cancel_goal"}) {
    const std::string ending(suffix);
    if (service.size() > ending.size() &&
        service.compare(service.size() - ending.size(), ending.size(), ending) == 0) {
      return service.substr(0, service.size() - ending.size());
    }
  }
  return service;
}

std::string action_name_from_event_topic(const std::string& topic) {
  const std::string suffix = "/_action/event";
  if (topic.size() > suffix.size() &&
      topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) == 0) {
    return topic.substr(0, topic.size() - suffix.size());
  }
  return topic;
}

std::vector<std::uint8_t> encode_rpc_request(const std::string& service, std::type_index type,
                                             const void* request) {
  if (type == typeid(SendGoalRequest)) {
    const auto* goal_request = static_cast<const SendGoalRequest*>(request);
    const ActionCodec* codec = find_action_codec(action_name_from_service(service));
    if (goal_request == nullptr || codec == nullptr) {
      return {};
    }
    const std::vector<std::uint8_t> goal = codec->encode_goal(goal_request->goal);
    Writer writer;
    writer.str(goal_request->goal_id);
    writer.u32(static_cast<std::uint32_t>(goal.size()));
    writer.bytes(goal);
    return std::move(writer.data);
  }
  if (type == typeid(CancelGoalRequest)) {
    const auto* cancel_request = static_cast<const CancelGoalRequest*>(request);
    if (cancel_request == nullptr) {
      return {};
    }
    Writer writer;
    writer.str(cancel_request->goal_id);
    return std::move(writer.data);
  }
  return encode_rpc_value(type, request);
}

std::any decode_rpc_request(const std::string& service, const std::string& type_name,
                            const std::uint8_t* data, std::size_t size) {
  if (type_name == "krosc/action/SendGoalRequest") {
    const ActionCodec* codec = find_action_codec(action_name_from_service(service));
    if (codec == nullptr) {
      return {};
    }
    Reader reader(data, size);
    SendGoalRequest request;
    request.goal_id = reader.str();
    const std::uint32_t length = reader.u32();
    if (!reader.ok() || length > reader.remaining()) {
      return {};
    }
    request.goal = codec->decode_goal(reader.rest(), length);
    if (!request.goal.has_value()) {
      return {};
    }
    return std::any(std::move(request));
  }
  if (type_name == "krosc/action/CancelGoalRequest") {
    Reader reader(data, size);
    CancelGoalRequest request;
    request.goal_id = reader.str();
    if (!reader.ok()) {
      return {};
    }
    return std::any(std::move(request));
  }
  return decode_rpc_value(type_name, data, size);
}

std::vector<std::uint8_t> encode_rpc_response(const std::string& service, std::type_index type,
                                              const void* response) {
  (void)service;
  return encode_rpc_value(type, response);
}

std::any decode_rpc_response(const std::string& service, const std::string& type_name,
                             const std::uint8_t* data, std::size_t size) {
  (void)service;
  return decode_rpc_value(type_name, data, size);
}

std::vector<std::uint8_t> encode_action_event(const std::string& topic,
                                              const ActionEvent& event) {
  const ActionCodec* codec = find_action_codec(action_name_from_event_topic(topic));
  if (codec == nullptr) {
    return {};
  }
  std::vector<std::uint8_t> feedback;
  std::vector<std::uint8_t> result;
  if (event.has_feedback) {
    feedback = codec->encode_feedback(event.feedback);
    if (feedback.empty()) {
      return {};
    }
  }
  if (event.has_result) {
    result = codec->encode_result(event.result);
    if (result.empty()) {
      return {};
    }
  }
  Writer writer;
  writer.str(event.goal_id);
  writer.u8(event.has_feedback ? 1U : 0U);
  writer.u8(event.has_result ? 1U : 0U);
  writer.u32(static_cast<std::uint32_t>(feedback.size()));
  writer.bytes(feedback);
  writer.u32(static_cast<std::uint32_t>(result.size()));
  writer.bytes(result);
  writer.u8(static_cast<std::uint8_t>(event.status));
  return std::move(writer.data);
}

std::optional<ActionEvent> decode_action_event(const std::string& topic,
                                               const std::uint8_t* data, std::size_t size) {
  const ActionCodec* codec = find_action_codec(action_name_from_event_topic(topic));
  if (codec == nullptr) {
    return std::nullopt;
  }
  Reader reader(data, size);
  ActionEvent event;
  event.goal_id = reader.str();
  event.has_feedback = reader.u8() != 0;
  event.has_result = reader.u8() != 0;
  const std::uint32_t feedback_length = reader.u32();
  if (!reader.ok() || feedback_length > reader.remaining()) {
    return std::nullopt;
  }
  const std::uint8_t* feedback = reader.rest();
  reader.skip(feedback_length);
  const std::uint32_t result_length = reader.u32();
  if (!reader.ok() || result_length > reader.remaining()) {
    return std::nullopt;
  }
  const std::uint8_t* result = reader.rest();
  reader.skip(result_length);
  if (event.has_feedback) {
    event.feedback = codec->decode_feedback(feedback, feedback_length);
    if (!event.feedback.has_value()) {
      return std::nullopt;
    }
  }
  if (event.has_result) {
    event.result = codec->decode_result(result, result_length);
    if (!event.result.has_value()) {
      return std::nullopt;
    }
  }
  event.status = static_cast<GoalStatus>(reader.u8());
  if (!reader.ok()) {
    return std::nullopt;
  }
  return event;
}

}  // namespace krosc::net
