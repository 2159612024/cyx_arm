#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "geometry_msgs/msg/point.hpp"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "random_numbers/random_numbers.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"

class WorkspaceVisualizer : public rclcpp::Node
{
public:
  WorkspaceVisualizer() : Node("cyx_workspace_visualizer")
  {
    group_ = declare_parameter("planning_group", "cyx_arm");
    end_effector_ = declare_parameter("end_effector_link", "tool0");
    sample_count_ = declare_parameter("sample_count", 50000);
    seed_ = declare_parameter("random_seed", 42);
    include_colliding_ = declare_parameter("include_colliding_samples", false);
    near_limit_ratio_ = declare_parameter("near_joint_limit_ratio", 0.05);
    point_size_ = declare_parameter("marker_point_size", 0.005);
    color_mode_ = declare_parameter("color_mode", "manipulability");
    output_csv_ = declare_parameter("output_csv", "");
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(
      "/cyx_arm/reachable_workspace", rclcpp::QoS(1).transient_local().reliable());
  }

  bool generate()
  {
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto model = loader.getModel();
    if (!model) {
      RCLCPP_ERROR(get_logger(), "Unable to load robot model");
      return false;
    }
    const auto * group = model->getJointModelGroup(group_);
    const auto * end_effector = model->getLinkModel(end_effector_);
    if (!group || !end_effector || sample_count_ <= 0) {
      RCLCPP_ERROR(
        get_logger(), "Invalid group, end-effector link, or sample count: %s %s %d",
        group_.c_str(), end_effector_.c_str(), sample_count_);
      return false;
    }

    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState state(model);
    random_numbers::RandomNumberGenerator rng(static_cast<boost::uint32_t>(seed_));
    std::vector<Sample> samples;
    samples.reserve(static_cast<std::size_t>(sample_count_));
    double max_manipulability = 0.0;
    std::size_t collision_count = 0;

    collision_detection::CollisionRequest request;
    request.group_name = group_;
    for (int index = 0; index < sample_count_; ++index) {
      state.setToRandomPositions(group, rng);
      state.update();
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result, state);
      if (result.collision) {
        ++collision_count;
        if (!include_colliding_) {
          continue;
        }
      }

      Sample sample;
      sample.colliding = result.collision;
      sample.near_limit = is_near_limit(state, model, group);
      const auto & transform = state.getGlobalLinkTransform(end_effector);
      sample.point.x = transform.translation().x();
      sample.point.y = transform.translation().y();
      sample.point.z = transform.translation().z();
      state.copyJointGroupPositions(group, sample.joints);

      Eigen::MatrixXd jacobian;
      if (state.getJacobian(group, end_effector, Eigen::Vector3d::Zero(), jacobian) &&
        jacobian.rows() > 0)
      {
        const double determinant = (jacobian * jacobian.transpose()).determinant();
        sample.manipulability = std::sqrt(std::max(0.0, determinant));
        max_manipulability = std::max(max_manipulability, sample.manipulability);
      }
      samples.push_back(std::move(sample));
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = model->getModelFrame();
    marker.header.stamp = now();
    marker.ns = "cyx_reachable_workspace";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = point_size_;
    marker.scale.y = point_size_;
    marker.points.reserve(samples.size());
    marker.colors.reserve(samples.size());
    for (const auto & sample : samples) {
      marker.points.push_back(sample.point);
      marker.colors.push_back(color(sample, max_manipulability));
    }
    publisher_->publish(marker);

    if (!output_csv_.empty() && !write_csv(output_csv_, group->getVariableNames(), samples)) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "Published %zu workspace points from %d samples (colliding=%zu filtered=%s, seed=%d, eef=%s)",
      samples.size(), sample_count_, collision_count,
      include_colliding_ ? "false" : "true", seed_, end_effector_.c_str());
    return true;
  }

private:
  struct Sample
  {
    geometry_msgs::msg::Point point;
    std::vector<double> joints;
    double manipulability{0.0};
    bool colliding{false};
    bool near_limit{false};
  };

  bool is_near_limit(
    const moveit::core::RobotState & state,
    const moveit::core::RobotModelConstPtr & model,
    const moveit::core::JointModelGroup * group) const
  {
    for (const auto & variable : group->getVariableNames()) {
      const auto & bounds = model->getVariableBounds(variable);
      if (!bounds.position_bounded_) {
        continue;
      }
      const double range = bounds.max_position_ - bounds.min_position_;
      const double value = state.getVariablePosition(variable);
      if (range > 0.0 &&
        (value - bounds.min_position_ <= near_limit_ratio_ * range ||
        bounds.max_position_ - value <= near_limit_ratio_ * range))
      {
        return true;
      }
    }
    return false;
  }

  std_msgs::msg::ColorRGBA color(const Sample & sample, double max_manipulability) const
  {
    std_msgs::msg::ColorRGBA result;
    result.a = 0.75F;
    if (sample.colliding) {
      result.r = 1.0F;
      return result;
    }
    if (sample.near_limit) {
      result.r = 1.0F;
      result.g = 0.8F;
      return result;
    }
    if (color_mode_ == "manipulability" && max_manipulability > 0.0) {
      const auto normalized = static_cast<float>(sample.manipulability / max_manipulability);
      result.r = 1.0F - normalized;
      result.g = normalized;
      result.b = 0.2F;
    } else {
      result.g = 1.0F;
    }
    return result;
  }

  bool write_csv(
    const std::string & path, const std::vector<std::string> & joint_names,
    const std::vector<Sample> & samples)
  {
    std::ofstream output(path);
    if (!output) {
      RCLCPP_ERROR(get_logger(), "Unable to open workspace CSV: %s", path.c_str());
      return false;
    }
    for (const auto & name : joint_names) {
      output << name << ',';
    }
    output << "x,y,z,colliding,near_joint_limit,manipulability\n";
    output << std::setprecision(10);
    for (const auto & sample : samples) {
      for (const double joint : sample.joints) {
        output << joint << ',';
      }
      output << sample.point.x << ',' << sample.point.y << ',' << sample.point.z << ',' <<
        sample.colliding << ',' << sample.near_limit << ',' << sample.manipulability << '\n';
    }
    RCLCPP_INFO(get_logger(), "Wrote workspace CSV: %s", path.c_str());
    return true;
  }

  std::string group_;
  std::string end_effector_;
  int sample_count_{50000};
  int seed_{42};
  bool include_colliding_{false};
  double near_limit_ratio_{0.05};
  double point_size_{0.005};
  std::string color_mode_;
  std::string output_csv_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WorkspaceVisualizer>();
  if (!node->generate()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
