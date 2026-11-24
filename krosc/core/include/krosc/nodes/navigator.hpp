#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "krosc/msgs.hpp"
#include "krosc/node.hpp"
#include "krosc/tf.hpp"
#include "krosc/tf_ros.hpp"

namespace krosc::nodes {

// nav2의 planner + controller 최소 버전:
//   goal_pose / goal_poses 를 받아 지도(map) 위에서 A*로 경로를 만들고 pure pursuit으로 따라간다.
//   /initialpose로 위치를 잡은 뒤(즉 map -> base_footprint TF가 있을 때)만 움직인다.
//   cmd_vel_nav로 내보내고, twist_mux가 teleop(cmd_vel_teleop)과 우선순위를 섞는다.
class Navigator final : public Node {
 public:
  explicit Navigator(Context& context, std::string name = "navigator",
                     std::map<std::string, ParamValue> parameters = {});
  ~Navigator() override = default;

  Navigator(const Navigator&) = delete;
  Navigator& operator=(const Navigator&) = delete;

  [[nodiscard]] bool active() const;
  [[nodiscard]] std::size_t path_size() const;

 private:
  struct Grid {
    int width = 0;
    int height = 0;
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::vector<std::uint8_t> cost;  // 0 free, 1..99 inflation, 255 unknown, 254 lethal
    [[nodiscard]] bool valid() const {
      return width > 0 && height > 0 && cost.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }
  };

  void on_map(const msgs::OccupancyGrid& map);
  void on_goal(const msgs::PoseStamped& goal);
  void on_goals(const msgs::Path& goals);
  void on_preview_goal(const msgs::PoseStamped& goal);
  void on_preview_goals(const msgs::Path& goals);
  void handle_goal(const msgs::PoseStamped& goal, bool preview);
  void handle_goals(const msgs::Path& goals, bool preview);
  void on_cancel(const msgs::Empty& message);
  void tick();

  [[nodiscard]] bool build_grid(const msgs::OccupancyGrid& map);
  [[nodiscard]] std::optional<msgs::Pose2D> robot_pose() const;
  [[nodiscard]] std::optional<msgs::Pose2D> to_map(const msgs::PoseStamped& pose) const;
  [[nodiscard]] bool plan(const Grid& grid, const msgs::Pose2D& start, const msgs::Pose2D& goal,
                          std::vector<msgs::Pose2D>& path) const;
  [[nodiscard]] int nearest_free(const Grid& grid, int cell) const;
  void publish_plan(const std::vector<msgs::Pose2D>& path);
  void publish_command(double linear, double angular);
  void publish_status(const std::string& text);
  void publish_global_costmap();
  void publish_local_costmap();
  void publish_empty_costmaps();
  void stop(const std::string& status);

  Parameter<std::string> map_topic_;
  Parameter<std::string> goal_topic_;
  Parameter<std::string> goals_topic_;
  Parameter<std::string> preview_goal_topic_;
  Parameter<std::string> preview_goals_topic_;
  Parameter<std::string> cancel_topic_;
  Parameter<std::string> plan_topic_;
  Parameter<std::string> status_topic_;
  Parameter<std::string> cmd_vel_topic_;
  Parameter<std::string> global_costmap_topic_;
  Parameter<std::string> local_costmap_topic_;
  Parameter<std::string> map_frame_;
  Parameter<std::string> robot_frame_;
  Parameter<double> controller_frequency_;
  Parameter<double> vx_max_;
  Parameter<double> wz_max_;
  Parameter<double> lookahead_;
  Parameter<double> inflation_;
  Parameter<double> xy_tolerance_;
  Parameter<double> yaw_tolerance_;
  Parameter<double> local_costmap_size_;
  Parameter<double> costmap_period_;

  Buffer buffer_;
  std::unique_ptr<TransformListener> listener_;
  mutable std::mutex mutex_;
  std::optional<Grid> grid_;
  std::vector<msgs::Pose2D> path_;
  std::vector<msgs::Pose2D> waypoints_;
  std::vector<std::size_t> waypoint_indices_;
  std::size_t waypoint_index_ = 0;
  bool active_ = false;
  std::size_t path_index_ = 0;
  double last_costmap_ = -1e18;
  std::string last_status_;
};

}  // namespace krosc::nodes
