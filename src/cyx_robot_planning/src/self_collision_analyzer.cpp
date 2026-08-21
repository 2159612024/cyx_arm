#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "moveit/collision_detection/collision_matrix.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "random_numbers/random_numbers.h"
#include "rclcpp/rclcpp.hpp"

class SelfCollisionAnalyzer : public rclcpp::Node
{
public:
  SelfCollisionAnalyzer() : Node("cyx_self_collision_analyzer")
  {
    group_ = declare_parameter("planning_group", "cyx_arm");
    sample_count_ = declare_parameter("sample_count", 10000);
    seed_ = declare_parameter("random_seed", 42);
    use_acm_ = declare_parameter("use_allowed_collision_matrix", true);
    near_limit_ratio_ = declare_parameter("near_joint_limit_ratio", 0.05);
    calculate_distance_ = declare_parameter("calculate_distance", true);
    strict_ = declare_parameter("strict", false);
    max_collision_ratio_ = declare_parameter("max_collision_ratio", 1.0);
    output_file_ = declare_parameter("output_file", "");
  }

  bool run()
  {
    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto model = loader.getModel();
    if (!model) {
      RCLCPP_ERROR(get_logger(), "Unable to load robot model");
      return false;
    }
    const auto * group = model->getJointModelGroup(group_);
    if (!group) {
      RCLCPP_ERROR(get_logger(), "Unknown planning group: %s", group_.c_str());
      return false;
    }
    if (sample_count_ <= 0) {
      RCLCPP_ERROR(get_logger(), "sample_count must be positive");
      return false;
    }

    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    random_numbers::RandomNumberGenerator rng(static_cast<boost::uint32_t>(seed_));

    const auto & collision_links = model->getLinkModelNamesWithCollisionGeometry();
    collision_detection::AllowedCollisionMatrix raw_acm(collision_links, false);
    const auto & active_acm = use_acm_ ? scene.getAllowedCollisionMatrix() : raw_acm;

    collision_detection::CollisionRequest request;
    request.group_name = group_;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 1;

    std::map<std::pair<std::string, std::string>, std::size_t> pair_counts;
    std::size_t colliding_samples = 0;
    std::size_t near_limit_samples = 0;
    double minimum_distance = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample < sample_count_; ++sample) {
      state.setToRandomPositions(group, rng);
      state.update();
      bool near_limit = false;
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
          near_limit = true;
        }
      }
      if (near_limit) {
        ++near_limit_samples;
      }
      collision_detection::CollisionResult result;
      scene.checkSelfCollision(request, result, state, active_acm);
      if (calculate_distance_) {
        const double distance = scene.distanceToCollisionUnpadded(state, active_acm);
        if (std::isfinite(distance) && distance < std::numeric_limits<double>::max()) {
          minimum_distance = std::min(minimum_distance, distance);
        }
      }
      if (!result.collision) {
        continue;
      }
      ++colliding_samples;
      for (const auto & contact : result.contacts) {
        auto pair = contact.first;
        if (pair.second < pair.first) {
          std::swap(pair.first, pair.second);
        }
        ++pair_counts[pair];
      }
    }

    std::set<std::pair<std::string, std::string>> observed;
    for (const auto & item : pair_counts) {
      observed.insert(item.first);
    }

    RCLCPP_INFO(
      get_logger(),
      "samples=%d colliding=%zu collision_ratio=%.6f near_limit=%zu "
      "near_limit_ratio=%.6f minimum_distance=%.6f seed=%d acm=%s",
      sample_count_, colliding_samples,
      static_cast<double>(colliding_samples) / static_cast<double>(sample_count_),
      near_limit_samples,
      static_cast<double>(near_limit_samples) / static_cast<double>(sample_count_),
      std::isfinite(minimum_distance) ? minimum_distance : -1.0,
      seed_, use_acm_ ? "enabled" : "disabled");
    for (const auto & item : pair_counts) {
      RCLCPP_INFO(
        get_logger(), "pair=%s,%s count=%zu ratio=%.6f",
        item.first.first.c_str(), item.first.second.c_str(), item.second,
        static_cast<double>(item.second) / static_cast<double>(sample_count_));
    }

    if (!use_acm_) {
      RCLCPP_INFO(get_logger(), "Never-observed collision pairs (sampling candidates only):");
      for (std::size_t i = 0; i < collision_links.size(); ++i) {
        for (std::size_t j = i + 1; j < collision_links.size(); ++j) {
          auto pair = std::make_pair(collision_links[i], collision_links[j]);
          if (pair.second < pair.first) {
            std::swap(pair.first, pair.second);
          }
          if (observed.count(pair) == 0) {
            RCLCPP_INFO(
              get_logger(), "never=%s,%s",
              pair.first.c_str(), pair.second.c_str());
          }
        }
      }
    }

    if (!output_file_.empty()) {
      std::ofstream output(output_file_);
      if (!output) {
        RCLCPP_ERROR(get_logger(), "Unable to open output file: %s", output_file_.c_str());
        return false;
      }
      output << "link1,link2,collision_count,sample_count,collision_ratio\n";
      output << std::setprecision(10);
      for (const auto & item : pair_counts) {
        output << item.first.first << ',' << item.first.second << ',' << item.second << ',' <<
          sample_count_ << ',' <<
          static_cast<double>(item.second) / static_cast<double>(sample_count_) << '\n';
      }
      RCLCPP_INFO(get_logger(), "Wrote collision report: %s", output_file_.c_str());
    }
    const double collision_ratio =
      static_cast<double>(colliding_samples) / static_cast<double>(sample_count_);
    if (strict_ && collision_ratio > max_collision_ratio_) {
      RCLCPP_ERROR(
        get_logger(), "Collision ratio %.6f exceeds strict limit %.6f",
        collision_ratio, max_collision_ratio_);
      return false;
    }
    return true;
  }

private:
  std::string group_;
  int sample_count_{10000};
  int seed_{42};
  bool use_acm_{true};
  double near_limit_ratio_{0.05};
  bool calculate_distance_{true};
  bool strict_{false};
  double max_collision_ratio_{1.0};
  std::string output_file_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SelfCollisionAnalyzer>();
  const bool success = node->run();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
