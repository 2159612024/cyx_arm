#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cyx_robot_interfaces/action/execute_plan.hpp"
#include "cyx_robot_interfaces/action/follow_cartesian_path.hpp"
#include "cyx_robot_interfaces/action/move_to_joints.hpp"
#include "cyx_robot_interfaces/action/move_to_named.hpp"
#include "cyx_robot_interfaces/action/move_to_pose.hpp"
#include "cyx_robot_interfaces/msg/cartesian_jog.hpp"
#include "cyx_robot_interfaces/msg/joint_jog.hpp"
#include "cyx_robot_interfaces/msg/motion_options.hpp"
#include "cyx_robot_interfaces/msg/motion_status.hpp"
#include "cyx_robot_interfaces/srv/get_motion_state.hpp"
#include "cyx_robot_interfaces/srv/plan.hpp"
#include "cyx_robot_interfaces/srv/stop_motion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class MotionServer : public rclcpp::Node
{
public:
  using Options = cyx_robot_interfaces::msg::MotionOptions;
  using Status = cyx_robot_interfaces::msg::MotionStatus;
  using MoveToNamed = cyx_robot_interfaces::action::MoveToNamed;
  using MoveToJoints = cyx_robot_interfaces::action::MoveToJoints;
  using MoveToPose = cyx_robot_interfaces::action::MoveToPose;
  using FollowCartesian = cyx_robot_interfaces::action::FollowCartesianPath;
  using ExecutePlan = cyx_robot_interfaces::action::ExecutePlan;

  MotionServer() : Node("cyx_motion_server")
  {
    group_ = declare_parameter("planning_group", "cyx_arm");
    default_eef_ = declare_parameter("default_end_effector_link", "tool0");
    default_frame_ = declare_parameter("default_reference_frame", "base_link");
    default_planner_ = declare_parameter("default_planner_id", "RRTConnectkConfigDefault");
    planning_time_ = declare_parameter("planning_time", 5.0);
    planning_attempts_ = declare_parameter("planning_attempts", 5);
    velocity_scaling_ = declare_parameter("velocity_scaling", 0.2);
    acceleration_scaling_ = declare_parameter("acceleration_scaling", 0.2);
    start_tolerance_ = declare_parameter("allowed_start_tolerance", 0.05);
    cartesian_step_ = declare_parameter("cartesian_eef_step", 0.005);
    cartesian_fraction_ = declare_parameter("cartesian_minimum_fraction", 0.98);
    jog_step_ = declare_parameter("jog_step_size", 0.010);
    jog_eef_step_ = declare_parameter("jog_eef_step", 0.002);
    jog_fraction_ = declare_parameter("jog_minimum_fraction", 0.99);
    cache_size_ = declare_parameter("plan_cache_size", 10);
    cache_timeout_ = declare_parameter("plan_cache_timeout", 300.0);
    set_status(Status::IDLE, "ready", 0.0);
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), group_);
    move_group_->setEndEffectorLink(default_eef_);
    move_group_->setPoseReferenceFrame(default_frame_);

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    named_server_ = rclcpp_action::create_server<MoveToNamed>(
      this, "/cyx_arm/move_to_named",
      std::bind(&MotionServer::goal_named, this, _1, _2),
      std::bind(&MotionServer::cancel_any<MoveToNamed>, this, _1),
      std::bind(&MotionServer::accept_named, this, _1));
    joints_server_ = rclcpp_action::create_server<MoveToJoints>(
      this, "/cyx_arm/move_to_joints",
      std::bind(&MotionServer::goal_joints, this, _1, _2),
      std::bind(&MotionServer::cancel_any<MoveToJoints>, this, _1),
      std::bind(&MotionServer::accept_joints, this, _1));
    pose_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "/cyx_arm/move_to_pose",
      std::bind(&MotionServer::goal_pose, this, _1, _2),
      std::bind(&MotionServer::cancel_any<MoveToPose>, this, _1),
      std::bind(&MotionServer::accept_pose, this, _1));
    cartesian_server_ = rclcpp_action::create_server<FollowCartesian>(
      this, "/cyx_arm/follow_cartesian",
      std::bind(&MotionServer::goal_cartesian, this, _1, _2),
      std::bind(&MotionServer::cancel_any<FollowCartesian>, this, _1),
      std::bind(&MotionServer::accept_cartesian, this, _1));
    execute_server_ = rclcpp_action::create_server<ExecutePlan>(
      this, "/cyx_arm/execute",
      std::bind(&MotionServer::goal_execute, this, _1, _2),
      std::bind(&MotionServer::cancel_any<ExecutePlan>, this, _1),
      std::bind(&MotionServer::accept_execute, this, _1));

    plan_service_ = create_service<cyx_robot_interfaces::srv::Plan>(
      "/cyx_arm/plan", std::bind(&MotionServer::plan_service, this, _1, _2));
    stop_service_ = create_service<cyx_robot_interfaces::srv::StopMotion>(
      "/cyx_arm/stop", std::bind(&MotionServer::stop_service, this, _1, _2));
    state_service_ = create_service<cyx_robot_interfaces::srv::GetMotionState>(
      "/cyx_arm/get_state", std::bind(&MotionServer::state_service, this, _1, _2));
    jog_sub_ = create_subscription<cyx_robot_interfaces::msg::CartesianJog>(
      "/cyx_arm/cartesian_jog", 10, std::bind(&MotionServer::jog_callback, this, _1));
    joint_jog_sub_ = create_subscription<cyx_robot_interfaces::msg::JointJog>(
      "/cyx_arm/joint_jog", 10, std::bind(&MotionServer::joint_jog_callback, this, _1));
    RCLCPP_INFO(get_logger(), "CYX motion server ready (group=%s, eef=%s)", group_.c_str(), default_eef_.c_str());
  }

private:
  struct CachedPlan
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    std::vector<double> start;
    rclcpp::Time created;
    std::string eef;
  };

  bool reserve(const std::string & id)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) return false;
    active_id_ = id;
    cancel_requested_ = false;
    return true;
  }

  void release()
  {
    busy_ = false;
    active_id_.clear();
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_.active_request_id.clear();
    }
    cyx_robot_interfaces::msg::JointJog::SharedPtr pending;
    {
      std::lock_guard<std::mutex> lock(pending_jog_mutex_);
      pending = pending_joint_jog_;
      pending_joint_jog_.reset();
    }
    if (pending) {
      RCLCPP_INFO(get_logger(), "Starting latest queued joint jog");
      joint_jog_callback(pending);
    }
  }

  std::string next_id()
  {
    return "plan_" + std::to_string(++sequence_) + "_" + std::to_string(now().nanoseconds());
  }

  void set_status(uint8_t state, const std::string & message, double progress)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.state = state;
    status_.active_request_id = active_id_;
    status_.message = message;
    status_.progress = progress;
    if (state == Status::FAILED) last_error_ = message;
    else if (state == Status::SUCCEEDED) last_error_.clear();
  }

  Status status_copy()
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
  }

  Options normalized(const Options & in)
  {
    Options o = in;
    if (o.planning_group.empty()) o.planning_group = group_;
    if (o.end_effector_link.empty()) o.end_effector_link = default_eef_;
    if (o.reference_frame.empty()) o.reference_frame = default_frame_;
    if (o.planner_id.empty()) o.planner_id = default_planner_;
    if (o.planning_time <= 0.0) o.planning_time = planning_time_;
    if (o.planning_attempts <= 0) o.planning_attempts = planning_attempts_;
    if (o.velocity_scaling <= 0.0) o.velocity_scaling = velocity_scaling_;
    if (o.acceleration_scaling <= 0.0) o.acceleration_scaling = acceleration_scaling_;
    return o;
  }

  bool finite(double v) const {return std::isfinite(v);}

  bool configure(const Options & raw, std::string & error)
  {
    auto o = normalized(raw);
    if (o.planning_group != group_) {error = "unsupported planning group: " + o.planning_group; return false;}
    if (o.velocity_scaling <= 0.0 || o.velocity_scaling > 1.0 ||
      o.acceleration_scaling <= 0.0 || o.acceleration_scaling > 1.0)
    {error = "velocity/acceleration scaling must be in (0, 1]"; return false;}
    auto model = move_group_->getRobotModel();
    if (!model->hasLinkModel(o.end_effector_link)) {error = "unknown end-effector link: " + o.end_effector_link; return false;}
    move_group_->setEndEffectorLink(o.end_effector_link);
    move_group_->setPoseReferenceFrame(o.reference_frame);
    move_group_->setPlannerId(o.planner_id);
    move_group_->setPlanningTime(o.planning_time);
    move_group_->setNumPlanningAttempts(o.planning_attempts);
    move_group_->setMaxVelocityScalingFactor(o.velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(o.acceleration_scaling);
    move_group_->setStartStateToCurrentState();
    return true;
  }

  bool plan_current(moveit::planning_interface::MoveGroupInterface::Plan & plan, std::string & error)
  {
    auto code = move_group_->plan(plan);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "MoveIt planning failed (code " + std::to_string(code.val) + ")";
      return false;
    }
    return true;
  }

  std::string cache_plan(const moveit::planning_interface::MoveGroupInterface::Plan & plan, const std::string & eef)
  {
    std::vector<double> start;
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (!trajectory.points.empty()) start = trajectory.points.front().positions;
    std::lock_guard<std::mutex> lock(cache_mutex_);
    while (static_cast<int>(cache_.size()) >= cache_size_) cache_.pop_front();
    const auto id = next_id();
    cache_.push_back({plan, start, now(), eef});
    cache_ids_.push_back(id);
    while (cache_ids_.size() > cache_.size()) cache_ids_.pop_front();
    return id;
  }

  bool get_cached(const std::string & id, CachedPlan & out, std::string & error)
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = std::find(cache_ids_.begin(), cache_ids_.end(), id);
    if (it == cache_ids_.end()) {error = "unknown plan_id"; return false;}
    const auto index = static_cast<size_t>(std::distance(cache_ids_.begin(), it));
    out = cache_.at(index);
    if ((now() - out.created).seconds() > cache_timeout_) {error = "cached plan expired"; return false;}
    auto state = move_group_->getCurrentState(1.0);
    if (!state) {error = "current robot state unavailable"; return false;}
    std::vector<double> current;
    state->copyJointGroupPositions(group_, current);
    if (current.size() != out.start.size()) {error = "cached plan start state is invalid"; return false;}
    for (size_t i = 0; i < current.size(); ++i) {
      if (std::abs(current[i] - out.start[i]) > start_tolerance_) {
        error = "current state differs from cached plan start"; return false;
      }
    }
    return true;
  }

  bool execute(const moveit::planning_interface::MoveGroupInterface::Plan & plan, std::string & error)
  {
    set_status(Status::EXECUTING, "executing", 0.7);
    auto code = move_group_->execute(plan);
    if (cancel_requested_) {error = "motion canceled"; return false;}
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "trajectory execution failed (code " + std::to_string(code.val) + ")"; return false;
    }
    return true;
  }

  bool parameterize(moveit_msgs::msg::RobotTrajectory & message, const Options & raw, std::string & error)
  {
    auto model = move_group_->getRobotModel();
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    const auto & source = message.joint_trajectory;
    if (source.points.empty() || source.joint_names.size() != source.points.front().positions.size()) {
      error = "Cartesian trajectory has no valid start state"; return false;
    }
    for (size_t i = 0; i < source.joint_names.size(); ++i) {
      state.setVariablePosition(source.joint_names[i], source.points.front().positions[i]);
    }
    state.update();
    robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(), group_);
    trajectory.setRobotTrajectoryMsg(state, message);
    const auto o = normalized(raw);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, o.velocity_scaling, o.acceleration_scaling)) {
      error = "trajectory time parameterization failed"; return false;
    }
    trajectory.getRobotTrajectoryMsg(message);
    return true;
  }

  bool set_joint_target(const std::vector<std::string> & names, const std::vector<double> & positions, std::string & error)
  {
    const auto expected = move_group_->getJointNames();
    if (positions.size() != expected.size()) {error = "exactly six joint positions are required"; return false;}
    for (double p : positions) if (!finite(p)) {error = "joint target contains NaN/Inf"; return false;}
    if (names.empty()) return move_group_->setJointValueTarget(positions);
    if (names.size() != positions.size()) {error = "joint_names and positions size mismatch"; return false;}
    std::map<std::string, double> target;
    for (size_t i = 0; i < names.size(); ++i) target[names[i]] = positions[i];
    if (!move_group_->setJointValueTarget(target)) {error = "joint target violates limits or names"; return false;}
    return true;
  }

  bool valid_pose(const geometry_msgs::msg::PoseStamped & p, std::string & error)
  {
    const auto & q = p.pose.orientation;
    if (!finite(p.pose.position.x) || !finite(p.pose.position.y) || !finite(p.pose.position.z) ||
      !finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w))
    {error = "pose contains NaN/Inf"; return false;}
    const double norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (std::abs(norm - 1.0) > 1e-3) {error = "pose quaternion must be normalized"; return false;}
    return true;
  }

  template<typename ActionT>
  rclcpp_action::GoalResponse reject_if_busy(const typename ActionT::Goal &)
  {
    return busy_ ? rclcpp_action::GoalResponse::REJECT : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::GoalResponse goal_named(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToNamed::Goal> g)
  {return reject_if_busy<MoveToNamed>(*g);}
  rclcpp_action::GoalResponse goal_joints(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToJoints::Goal> g)
  {return reject_if_busy<MoveToJoints>(*g);}
  rclcpp_action::GoalResponse goal_pose(const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal> g)
  {return reject_if_busy<MoveToPose>(*g);}
  rclcpp_action::GoalResponse goal_cartesian(const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowCartesian::Goal> g)
  {return reject_if_busy<FollowCartesian>(*g);}
  rclcpp_action::GoalResponse goal_execute(const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecutePlan::Goal> g)
  {return reject_if_busy<ExecutePlan>(*g);}

  template<typename ActionT>
  rclcpp_action::CancelResponse cancel_any(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)
  {
    cancel_requested_ = true;
    if (move_group_) move_group_->stop();
    set_status(Status::CANCELED, "cancel requested", 0.0);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accept_named(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> h)
  {std::thread([this,h]{run_named(h);}).detach();}
  void accept_joints(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> h)
  {std::thread([this,h]{run_joints(h);}).detach();}
  void accept_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> h)
  {std::thread([this,h]{run_pose(h);}).detach();}
  void accept_cartesian(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowCartesian>> h)
  {std::thread([this,h]{run_cartesian(h);}).detach();}
  void accept_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>> h)
  {std::thread([this,h]{run_execute(h);}).detach();}

  template<typename ActionT>
  void finish(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & h,
    const std::shared_ptr<typename ActionT::Result> & result, bool success, const std::string & message)
  {
    result->success = success;
    result->message = message;
    result->error_code = success ? moveit_msgs::msg::MoveItErrorCodes::SUCCESS : moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    if (cancel_requested_ || h->is_canceling()) h->canceled(result);
    else if (success) h->succeed(result);
    else h->abort(result);
    set_status(success ? Status::SUCCEEDED : (cancel_requested_ ? Status::CANCELED : Status::FAILED), message, success ? 1.0 : 0.0);
    release();
  }

  void run_named(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToNamed>> h)
  {
    auto result = std::make_shared<MoveToNamed::Result>(); std::string error;
    if (!reserve(next_id())) {result->message="motion server busy"; h->abort(result); return;}
    set_status(Status::PLANNING, "planning named pose", 0.1);
    const auto g=h->get_goal(); auto o=normalized(g->options);
    if (g->name != "home") {finish(h,result,false,"only named pose 'home' is currently supported"); return;}
    if (!configure(o,error) || !move_group_->setNamedTarget("home")) {finish(h,result,false,error.empty()?"home target unavailable":error); return;}
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!plan_current(plan,error)) {finish(h,result,false,error); return;}
    result->plan_id=cache_plan(plan,o.end_effector_link);
    if (!o.plan_only && !execute(plan,error)) {finish(h,result,false,error); return;}
    finish(h,result,true,o.plan_only?"home planned":"home reached");
  }

  void run_joints(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToJoints>> h)
  {
    auto result=std::make_shared<MoveToJoints::Result>(); std::string error;
    if (!reserve(next_id())) {result->message="motion server busy"; h->abort(result); return;}
    set_status(Status::PLANNING,"planning joint goal",0.1);
    const auto g=h->get_goal(); auto o=normalized(g->options);
    if (!configure(o,error) || !set_joint_target(g->joint_names,g->positions,error)) {finish(h,result,false,error); return;}
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!plan_current(plan,error)) {finish(h,result,false,error); return;}
    result->plan_id=cache_plan(plan,o.end_effector_link);
    if (!o.plan_only && !execute(plan,error)) {finish(h,result,false,error); return;}
    finish(h,result,true,o.plan_only?"joint goal planned":"joint goal reached");
  }

  void run_pose(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPose>> h)
  {
    auto result=std::make_shared<MoveToPose::Result>(); std::string error;
    if (!reserve(next_id())) {result->message="motion server busy"; h->abort(result); return;}
    set_status(Status::PLANNING,"planning pose goal",0.1);
    const auto g=h->get_goal(); auto o=normalized(g->options); auto target=g->target;
    if (target.header.frame_id.empty()) target.header.frame_id=o.reference_frame;
    if (!configure(o,error) || !valid_pose(target,error) || !move_group_->setPoseTarget(target,o.end_effector_link)) {finish(h,result,false,error.empty()?"pose has no IK solution":error); return;}
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!plan_current(plan,error)) {finish(h,result,false,error); return;}
    result->plan_id=cache_plan(plan,o.end_effector_link); move_group_->clearPoseTargets();
    if (!o.plan_only && !execute(plan,error)) {finish(h,result,false,error); return;}
    finish(h,result,true,o.plan_only?"pose goal planned":"pose goal reached");
  }

  void run_cartesian(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowCartesian>> h)
  {
    auto result=std::make_shared<FollowCartesian::Result>(); std::string error;
    if (!reserve(next_id())) {result->message="motion server busy"; h->abort(result); return;}
    set_status(Status::PLANNING,"planning Cartesian path",0.1);
    const auto g=h->get_goal(); auto o=normalized(g->options);
    if (!configure(o,error) || g->waypoints.empty()) {finish(h,result,false,error.empty()?"at least one waypoint is required":error); return;}
    std::vector<geometry_msgs::msg::Pose> poses;
    std::string frame=g->waypoints.front().header.frame_id.empty()?o.reference_frame:g->waypoints.front().header.frame_id;
    for (auto p:g->waypoints) {
      if (p.header.frame_id.empty()) p.header.frame_id=frame;
      if (p.header.frame_id!=frame || !valid_pose(p,error)) {finish(h,result,false,error.empty()?"all waypoints must use one frame":error); return;}
      poses.push_back(p.pose);
    }
    move_group_->setPoseReferenceFrame(frame);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double step=g->eef_step>0?g->eef_step:cartesian_step_;
    const double threshold=g->minimum_fraction>0?g->minimum_fraction:cartesian_fraction_;
    result->fraction=move_group_->computeCartesianPath(poses,step,g->jump_threshold,trajectory,true);
    if (result->fraction<threshold) {finish(h,result,false,"Cartesian path fraction below threshold"); return;}
    if (!parameterize(trajectory,o,error)) {finish(h,result,false,error); return;}
    moveit::planning_interface::MoveGroupInterface::Plan plan; plan.trajectory_=trajectory;
    result->plan_id=cache_plan(plan,o.end_effector_link);
    if (!o.plan_only && !execute(plan,error)) {finish(h,result,false,error); return;}
    finish(h,result,true,o.plan_only?"Cartesian path planned":"Cartesian path executed");
  }

  void run_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecutePlan>> h)
  {
    auto result=std::make_shared<ExecutePlan::Result>(); std::string error; CachedPlan cached;
    if (!reserve(next_id())) {result->message="motion server busy"; h->abort(result); return;}
    if (!get_cached(h->get_goal()->plan_id,cached,error) || !execute(cached.plan,error)) {finish(h,result,false,error); return;}
    finish(h,result,true,"cached plan executed");
  }

  void plan_service(const std::shared_ptr<cyx_robot_interfaces::srv::Plan::Request> req,
    std::shared_ptr<cyx_robot_interfaces::srv::Plan::Response> res)
  {
    if (!reserve(next_id())) {res->message="motion server busy"; return;}
    std::string error; auto o=normalized(req->options); o.plan_only=true;
    set_status(Status::PLANNING,"planning service request",0.1);
    bool target=false;
    if (configure(o,error)) {
      if (req->goal_type==req->JOINT_GOAL) target=set_joint_target(req->joint_names,req->joint_positions,error);
      else if (req->goal_type==req->NAMED_GOAL && req->named_target=="home") target=move_group_->setNamedTarget("home");
      else if (req->goal_type==req->POSE_GOAL) {
        auto pose=req->pose_target; if (pose.header.frame_id.empty()) pose.header.frame_id=o.reference_frame;
        target=valid_pose(pose,error) && move_group_->setPoseTarget(pose,o.end_effector_link);
      } else error="unsupported goal type or named target";
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (target && plan_current(plan,error)) {
      res->success=true; res->plan_id=cache_plan(plan,o.end_effector_link);
      res->trajectory=plan.trajectory_.joint_trajectory; res->error_code=moveit_msgs::msg::MoveItErrorCodes::SUCCESS; res->message="plan created";
    } else {res->error_code=moveit_msgs::msg::MoveItErrorCodes::FAILURE; res->message=error;}
    set_status(res->success ? Status::PLANNED : Status::FAILED,
      res->success ? "plan created" : error, res->success ? 1.0 : 0.0);
    move_group_->clearPoseTargets(); release();
  }

  void stop_service(const std::shared_ptr<cyx_robot_interfaces::srv::StopMotion::Request>,
    std::shared_ptr<cyx_robot_interfaces::srv::StopMotion::Response> res)
  {
    {
      std::lock_guard<std::mutex> lock(pending_jog_mutex_);
      pending_joint_jog_.reset();
    }
    cancel_requested_=true; move_group_->stop(); set_status(Status::CANCELED,"motion stopped",0.0);
    res->success=true; res->message="stop requested";
  }

  void state_service(const std::shared_ptr<cyx_robot_interfaces::srv::GetMotionState::Request>,
    std::shared_ptr<cyx_robot_interfaces::srv::GetMotionState::Response> res)
  {
    res->status=status_copy(); res->active_end_effector_link=move_group_->getEndEffectorLink();
    res->end_effector_pose=move_group_->getCurrentPose(res->active_end_effector_link);
    auto state=move_group_->getCurrentState(1.0);
    if (state) {
      res->joint_state.header.stamp=now(); res->joint_state.name=move_group_->getJointNames();
      state->copyJointGroupPositions(group_,res->joint_state.position); res->moveit_ready=true; res->controller_ready=true;
    }
    res->last_error=last_error_;
  }

  void jog_callback(const cyx_robot_interfaces::msg::CartesianJog::SharedPtr msg)
  {
    if (msg->translation.x==0.0 && msg->translation.y==0.0 && msg->translation.z==0.0) {
      {
        std::lock_guard<std::mutex> lock(pending_jog_mutex_);
        pending_joint_jog_.reset();
      }
      cancel_requested_=true; move_group_->stop(); return;
    }
    if (!reserve(next_id())) {RCLCPP_WARN(get_logger(),"Jog ignored: motion server busy"); return;}
    std::thread([this,msg]{
      std::string error; Options o; o.end_effector_link=msg->end_effector_link; o.reference_frame=msg->reference_frame;
      o.velocity_scaling=0.1; o.acceleration_scaling=0.1;
      set_status(Status::PLANNING,"planning position-only jog",0.1);
      if (!configure(o,error)) {set_status(Status::FAILED,error,0); release(); return;}
      auto current=move_group_->getCurrentPose(normalized(o).end_effector_link);
      if (!msg->header.frame_id.empty() && current.header.frame_id!=msg->header.frame_id) {
        set_status(Status::FAILED,"jog frame must match current MoveIt pose reference frame",0); release(); return;
      }
      const auto x = current.pose.position.x + msg->translation.x;
      const auto y = current.pose.position.y + msg->translation.y;
      const auto z = current.pose.position.z + msg->translation.z;
      if (!move_group_->setPositionTarget(x, y, z, normalized(o).end_effector_link)) {
        set_status(Status::FAILED,"unable to set jog position target",0); release(); return;
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned = plan_current(plan,error);
      move_group_->clearPoseTargets();
      if (!planned) {set_status(Status::FAILED,error,0); release(); return;}
      if (!execute(plan,error)) {set_status(cancel_requested_?Status::CANCELED:Status::FAILED,error,0); release(); return;}
      set_status(Status::SUCCEEDED,"position-only jog complete",1.0); release();
    }).detach();
  }

  void joint_jog_callback(const cyx_robot_interfaces::msg::JointJog::SharedPtr msg)
  {
    if (!std::isfinite(msg->delta) || msg->delta == 0.0) {
      RCLCPP_WARN(get_logger(), "Joint jog ignored: delta must be finite and non-zero");
      return;
    }
    if (!reserve(next_id())) {
      {
        std::lock_guard<std::mutex> lock(pending_jog_mutex_);
        pending_joint_jog_ = msg;
      }
      RCLCPP_INFO(get_logger(), "Motion server busy; latest joint jog queued");
      return;
    }
    std::thread([this, msg] {
      std::string error;
      Options options;
      options.velocity_scaling = 0.1;
      options.acceleration_scaling = 0.1;
      set_status(Status::PLANNING, "planning joint jog", 0.1);
      if (!configure(options, error)) {
        set_status(Status::FAILED, error, 0.0); release(); return;
      }
      auto positions = move_group_->getCurrentJointValues();
      if (msg->joint_index < 0 || static_cast<std::size_t>(msg->joint_index) >= positions.size()) {
        set_status(Status::FAILED, "joint jog index out of range", 0.0); release(); return;
      }
      positions[static_cast<std::size_t>(msg->joint_index)] += msg->delta;
      if (!move_group_->setJointValueTarget(positions)) {
        set_status(Status::FAILED, "joint jog target violates joint bounds", 0.0); release(); return;
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (!plan_current(plan, error)) {
        set_status(Status::FAILED, error, 0.0); release(); return;
      }
      set_status(Status::EXECUTING, "executing joint jog", 0.7);
      if (!execute(plan, error)) {
        set_status(cancel_requested_ ? Status::CANCELED : Status::FAILED, error, 0.0);
        release(); return;
      }
      set_status(Status::SUCCEEDED, "joint jog complete", 1.0);
      release();
    }).detach();
  }

  std::string group_,default_eef_,default_frame_,default_planner_,active_id_,last_error_;
  double planning_time_,velocity_scaling_,acceleration_scaling_,start_tolerance_,cartesian_step_,cartesian_fraction_,jog_step_,jog_eef_step_,jog_fraction_,cache_timeout_;
  int planning_attempts_,cache_size_;
  std::atomic<bool> busy_{false},cancel_requested_{false}; std::atomic<uint64_t> sequence_{0};
  std::mutex status_mutex_,cache_mutex_,pending_jog_mutex_; Status status_;
  cyx_robot_interfaces::msg::JointJog::SharedPtr pending_joint_jog_;
  std::deque<CachedPlan> cache_; std::deque<std::string> cache_ids_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Server<MoveToNamed>::SharedPtr named_server_;
  rclcpp_action::Server<MoveToJoints>::SharedPtr joints_server_;
  rclcpp_action::Server<MoveToPose>::SharedPtr pose_server_;
  rclcpp_action::Server<FollowCartesian>::SharedPtr cartesian_server_;
  rclcpp_action::Server<ExecutePlan>::SharedPtr execute_server_;
  rclcpp::Service<cyx_robot_interfaces::srv::Plan>::SharedPtr plan_service_;
  rclcpp::Service<cyx_robot_interfaces::srv::StopMotion>::SharedPtr stop_service_;
  rclcpp::Service<cyx_robot_interfaces::srv::GetMotionState>::SharedPtr state_service_;
  rclcpp::Subscription<cyx_robot_interfaces::msg::CartesianJog>::SharedPtr jog_sub_;
  rclcpp::Subscription<cyx_robot_interfaces::msg::JointJog>::SharedPtr joint_jog_sub_;
};

int main(int argc,char **argv)
{
  rclcpp::init(argc,argv); auto node=std::make_shared<MotionServer>(); node->initialize();
  rclcpp::executors::MultiThreadedExecutor executor; executor.add_node(node); executor.spin();
  rclcpp::shutdown(); return 0;
}
