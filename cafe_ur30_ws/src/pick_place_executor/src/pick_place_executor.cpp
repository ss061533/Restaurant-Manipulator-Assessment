// pick_place_executor
//
// Action server `pick_place` (cafe_interfaces/action/PickPlace).
//
// Sequence (every motion step is checked, every failure ends in a safe recovery):
//   open gripper -> plan to pre-grasp (above tray) -> Cartesian descent -> close gripper
//   -> VERIFY grasp (finger opening == tray width) -> Cartesian lift -> VERIFY still held
//   -> plan to above drop zone -> Cartesian descent -> open -> Cartesian retreat -> observe pose
//
// Gripper axis: the two fingers close along the tool-frame Y axis. The tool orientation is
// built so that tool-Y is the tray's SHORT axis, i.e. the gripper always clamps across the
// width of the tray whatever its yaw is.  Because the gripper is 180°-symmetric, a second yaw
// candidate (yaw + pi) is tried if the first one is not reachable (wrist limits).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "cafe_interfaces/action/pick_place.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

struct BusyGuard
{
  explicit BusyGuard(std::atomic<bool> & b) : flag(b) {}
  ~BusyGuard() { flag = false; }
  std::atomic<bool> & flag;
};
}  // namespace

class PickPlaceExecutor : public rclcpp::Node
{
public:
  using PickPlace = cafe_interfaces::action::PickPlace;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PickPlace>;
  using FJT = control_msgs::action::FollowJointTrajectory;
  using Trigger = std_srvs::srv::Trigger;
  using Pose = geometry_msgs::msg::Pose;

  PickPlaceExecutor()
  : Node("pick_place_executor",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
  }

  // Must be called once the node is already being spun (MoveGroupInterface needs callbacks).
  void init()
  {
    group_ = param<std::string>("planning_group", "ur_manipulator");
    ee_link_ = param<std::string>("ee_link", "gripper_tcp");
    table_z_ = param<double>("table_z", 0.75);
    grasp_height_ = param<double>("grasp_height", 0.03);
    approach_height_ = param<double>("approach_height", 0.15);
    clearance_height_ = param<double>("clearance_height", 0.30);
    place_margin_ = param<double>("place_margin", 0.01);
    vel_scale_ = param<double>("velocity_scaling", 0.4);
    acc_scale_ = param<double>("acceleration_scaling", 0.4);
    cart_vel_scale_ = param<double>("cartesian_velocity_scaling", 0.15);
    cart_acc_scale_ = param<double>("cartesian_acceleration_scaling", 0.15);
    carry_vel_ = param<double>("carry_velocity_scaling", 0.3);
    carry_acc_ = param<double>("carry_acceleration_scaling", 0.15);
    planning_time_ = param<double>("planning_time", 5.0);
    planning_attempts_ = param<int>("planning_attempts", 5);
    eef_step_ = param<double>("cartesian_step", 0.005);
    min_fraction_ = param<double>("min_cartesian_fraction", 0.95);
    open_q_ = param<double>("gripper_open_q", 0.075);
    close_q_ = param<double>("gripper_close_target_q", 0.01);
    close_time_ = param<double>("gripper_close_time", 2.0);
    settle_time_ = param<double>("grasp_settle_time", 0.6);
    min_finger_q_ = param<double>("min_finger_q", 0.015);
    width_tol_ = param<double>("grasp_width_tolerance", 0.025);
    default_width_ = param<double>("default_tray_width", 0.10);
    observe_joints_ = param<std::vector<double>>(
      "observe_joints", {1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0});
    left_joint_ = param<std::string>("finger_left_joint", "finger_left_joint");
    right_joint_ = param<std::string>("finger_right_joint", "finger_right_joint");
    gripper_action_ = param<std::string>(
      "gripper_action", "/gripper_controller/follow_joint_trajectory");
    wall_cap_ = param<double>("wall_timeout_cap", 300.0);
    param<bool>("simulate_grasp_failure", false);  // runtime-togglable test hook

    // Everything is launched at once, so wait until move_group is really up.
    auto scene_client = create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");
    while (rclcpp::ok() && !scene_client->wait_for_service(2s)) {
      RCLCPP_INFO(get_logger(), "Waiting for move_group ...");
    }

    mg_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), group_);
    mg_->setEndEffectorLink(ee_link_);
    mg_->setPlannerId("RRTConnectkConfigDefault");
    mg_->setPlanningTime(planning_time_);
    mg_->setNumPlanningAttempts(planning_attempts_);
    mg_->setGoalPositionTolerance(0.002);
    mg_->setGoalOrientationTolerance(0.01);
    frame_ = mg_->getPlanningFrame();

    addTable();

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(js_mtx_);
        for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
          js_[m->name[i]] = m->position[i];
        }
      });
    gripper_client_ = rclcpp_action::create_client<FJT>(this, gripper_action_);

    srv_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    observe_srv_ = create_service<Trigger>(
      "~/go_observe",
      std::bind(&PickPlaceExecutor::onGoObserve, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, srv_group_);

    server_ = rclcpp_action::create_server<PickPlace>(
      this, "~/pick_place",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const PickPlace::Goal>) {
        if (busy_.exchange(true)) {
          RCLCPP_WARN(get_logger(), "Rejecting goal: executor busy");
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](std::shared_ptr<GoalHandle> gh) {
        std::thread([this, gh]() { execute(gh); }).detach();
      });

    RCLCPP_INFO(get_logger(),
      "Ready. group=%s ee=%s frame=%s table_z=%.3f grasp_tcp_z=%.3f", group_.c_str(),
      ee_link_.c_str(), frame_.c_str(), table_z_, graspZ());
  }

private:
  // ------------------------------------------------------------------ params
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!has_parameter(name)) {
      declare_parameter<T>(name, def);
    }
    return get_parameter(name).get_value<T>();
  }

  double graspZ() const { return table_z_ + grasp_height_; }
  double approachZ() const { return graspZ() + approach_height_; }
  double clearanceZ() const { return table_z_ + clearance_height_; }
  double placeZ() const { return graspZ() + place_margin_; }

  // -------------------------------------------------------------------- poses
  // Tool pointing down (-Z world), tool-Y along the tray's short axis.
  //   R = Rz(yaw + pi) * Rx(pi)
  static Pose makePose(double x, double y, double z, double yaw)
  {
    tf2::Quaternion qz, qx, q;
    qz.setRPY(0, 0, yaw + kPi);
    qx.setRPY(kPi, 0, 0);
    q = qz * qx;
    q.normalize();
    Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
    return p;
  }

  static double yawOf(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // ---------------------------------------------------------------- planning
  void addTable()
  {
    // Slightly below the real top (5 mm) so the robot base, which stands on the table,
    // is not reported as "in collision" with it.
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = frame_;
    obj.id = "table";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    const double top = table_z_ - 0.005;
    box.dimensions = {1.9, 2.0, 0.10};
    Pose p;
    p.position.x = 0.45;
    p.position.y = 0.0;
    p.position.z = top - 0.05;
    p.orientation.w = 1.0;
    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(p);
    obj.operation = obj.ADD;
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(obj);
  }

  // OMPL (random but complete) - only used as a fallback.
  bool omplGo(const Pose & target, const std::string & label)
  {
    mg_->setStartStateToCurrentState();
    mg_->setMaxVelocityScalingFactor(vel_scale_);
    mg_->setMaxAccelerationScalingFactor(acc_scale_);
    mg_->setPoseTarget(target, ee_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = (mg_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool ok = false;
    if (planned) {
      ok = (mg_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!ok) RCLCPP_WARN(get_logger(), "[%s] OMPL execution failed", label.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "[%s] OMPL planning failed", label.c_str());
    }
    mg_->clearPoseTargets();
    return ok;
  }

  bool executeWithTotg(robot_trajectory::RobotTrajectory & rt, double vs, double as,
    const std::string & label)
  {
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vs, as)) {
      RCLCPP_WARN(get_logger(), "[%s] time parameterisation failed", label.c_str());
      return false;
    }
    moveit_msgs::msg::RobotTrajectory msg;
    rt.getRobotTrajectoryMsg(msg);
    const bool ok = (mg_->execute(msg) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) RCLCPP_WARN(get_logger(), "[%s] execution failed", label.c_str());
    return ok;
  }

  // Straight Cartesian line (tool orientation interpolated) - deterministic, keeps the tool level.
  bool cartesian(const std::vector<Pose> & wps, const std::string & label, double vs = -1.0,
    double as = -1.0)
  {
    if (vs < 0) vs = cart_vel_scale_;
    if (as < 0) as = cart_acc_scale_;
    moveit_msgs::msg::RobotTrajectory traj;
    const double frac = mg_->computeCartesianPath(wps, eef_step_, 0.0, traj, true);
    if (frac < min_fraction_) {
      RCLCPP_WARN(get_logger(), "[%s] Cartesian path only %.0f%% feasible", label.c_str(),
        frac * 100.0);
      return false;
    }
    robot_trajectory::RobotTrajectory rt(mg_->getRobotModel(), group_);
    rt.setRobotTrajectoryMsg(*mg_->getCurrentState(2.0), traj);
    return executeWithTotg(rt, vs, as, label);
  }

  // Straight line in JOINT space (PTP): smooth and repeatable. Every sample is collision checked
  // against a local planning scene (robot SRDF + table), so no random planner is needed.
  bool jointLinear(const std::vector<double> & goal, const std::string & label, double vs,
    double as)
  {
    auto start = mg_->getCurrentState(3.0);
    if (!start) return false;
    const auto * jmg = start->getJointModelGroup(group_);
    std::vector<double> q0;
    start->copyJointGroupPositions(jmg, q0);
    if (q0.size() != goal.size()) return false;
    double maxd = 0.0;
    for (size_t i = 0; i < q0.size(); ++i) maxd = std::max(maxd, std::fabs(goal[i] - q0[i]));
    if (maxd < 1e-3) return true;
    const int n = std::max(10, static_cast<int>(std::ceil(maxd / 0.02)));

    planning_scene::PlanningScene scene(mg_->getRobotModel());
    Eigen::Isometry3d tp = Eigen::Isometry3d::Identity();
    tp.translation() = Eigen::Vector3d(0.45, 0.0, table_z_ - 0.005 - 0.05);
    scene.getWorldNonConst()->addToObject("table", std::make_shared<shapes::Box>(1.9, 2.0, 0.10), tp);

    robot_trajectory::RobotTrajectory rt(mg_->getRobotModel(), group_);
    moveit::core::RobotState s(*start);
    std::vector<double> q(q0.size());
    for (int k = 0; k <= n; ++k) {
      const double a = static_cast<double>(k) / n;
      for (size_t i = 0; i < q.size(); ++i) q[i] = q0[i] + a * (goal[i] - q0[i]);
      s.setJointGroupPositions(jmg, q);
      s.update();
      if (!scene.isStateValid(s, group_)) {
        RCLCPP_WARN(get_logger(), "[%s] joint-space line in collision at %.0f%%", label.c_str(),
          a * 100.0);
        return false;
      }
      rt.addSuffixWayPoint(s, 0.0);
    }
    return executeWithTotg(rt, vs, as, label);
  }

  bool ikJointMove(const Pose & target, const std::string & label, double vs, double as)
  {
    auto st = mg_->getCurrentState(3.0);
    if (!st) return false;
    moveit::core::RobotState goal(*st);   // seeded with the current joints -> nearest IK branch
    const auto * jmg = goal.getJointModelGroup(group_);
    if (!goal.setFromIK(jmg, target, ee_link_, 0.2)) {
      RCLCPP_WARN(get_logger(), "[%s] no IK solution", label.c_str());
      return false;
    }
    std::vector<double> q;
    goal.copyJointGroupPositions(jmg, q);
    return jointLinear(q, label, vs, as);
  }

  // Preferred order:  free move -> joint-space line;  carrying -> Cartesian line (tray stays level,
  // slow, low acceleration) -> joint line.  OMPL only if both fail.
  bool planAndGo(const Pose & target, const std::string & label, bool carrying = false)
  {
    if (carrying) {
      if (cartesian({target}, label + "/cartesian", carry_vel_, carry_acc_)) return true;
      if (ikJointMove(target, label + "/joint", carry_vel_, carry_acc_)) return true;
    } else {
      if (ikJointMove(target, label + "/joint", vel_scale_, acc_scale_)) return true;
    }
    RCLCPP_WARN(get_logger(), "[%s] falling back to OMPL", label.c_str());
    return omplGo(target, label);
  }

  bool goObserve()
  {
    if (jointLinear(observe_joints_, "observe", vel_scale_, acc_scale_)) return true;
    mg_->setStartStateToCurrentState();
    mg_->setMaxVelocityScalingFactor(vel_scale_);
    mg_->setMaxAccelerationScalingFactor(acc_scale_);
    mg_->setJointValueTarget(observe_joints_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "planning to observe pose failed");
      return false;
    }
    return mg_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ---- waiting helpers: timeouts are measured in ROS (sim) time so a slow simulator
  // (real-time factor < 1) does not cause false failures; wall_cap_ is only a safety net.
  template<typename FutureT>
  bool waitReady(FutureT & fut, double ros_timeout)
  {
    const auto t0 = now();
    const auto w0 = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      if (fut.wait_for(50ms) == std::future_status::ready) return true;
      if ((now() - t0).seconds() > ros_timeout) return false;
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count() > wall_cap_) {
        return false;
      }
    }
    return false;
  }

  void simSleep(double seconds)
  {
    const auto t0 = now();
    const auto w0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (now() - t0).seconds() < seconds) {
      std::this_thread::sleep_for(10ms);
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - w0).count() > wall_cap_) {
        break;
      }
    }
  }

  // ----------------------------------------------------------------- gripper
  bool moveGripper(double q, double duration)
  {
    if (!gripper_client_->wait_for_action_server(3s)) {
      RCLCPP_ERROR(get_logger(), "gripper action server not available");
      return false;
    }
    FJT::Goal goal;
    goal.trajectory.joint_names = {left_joint_, right_joint_};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {q, q};
    pt.time_from_start = rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points.push_back(pt);

    auto gh_future = gripper_client_->async_send_goal(goal);
    if (!waitReady(gh_future, 5.0)) return false;
    auto gh = gh_future.get();
    if (!gh) return false;
    auto res_future = gripper_client_->async_get_result(gh);
    // The result code is intentionally ignored: when the tray blocks the fingers the controller
    // may report a tolerance violation, which is normal. We verify the grasp from joint states.
    return waitReady(res_future, duration + 5.0);
  }

  bool fingers(double & ql, double & qr)
  {
    std::lock_guard<std::mutex> lk(js_mtx_);
    auto l = js_.find(left_joint_), r = js_.find(right_joint_);
    if (l == js_.end() || r == js_.end()) return false;
    ql = l->second;
    qr = r->second;
    return true;
  }

  bool openGripper() { return moveGripper(open_q_, 1.0); }

  bool closeGripper()
  {
    const bool ok = moveGripper(close_q_, close_time_);
    simSleep(settle_time_);
    return ok;
  }

  // Finger opening must equal the tray width: closed fully => missed, too wide => wrong object.
  bool graspHeld(double expected_width, std::string & info)
  {
    if (get_parameter("simulate_grasp_failure").as_bool()) {
      info = "simulated grasp failure (test hook)";
      return false;
    }
    double ql = 0, qr = 0;
    if (!fingers(ql, qr)) {
      info = "no finger joint states";
      return false;
    }
    const double width = ql + qr;
    char buf[160];
    if (std::min(ql, qr) < min_finger_q_) {
      std::snprintf(buf, sizeof(buf), "fingers closed fully (q=%.3f/%.3f): nothing grasped", ql, qr);
      info = buf;
      return false;
    }
    if (std::fabs(width - expected_width) > width_tol_) {
      std::snprintf(buf, sizeof(buf), "finger opening %.3f m != tray width %.3f m", width,
        expected_width);
      info = buf;
      return false;
    }
    std::snprintf(buf, sizeof(buf), "grasp ok (opening %.3f m)", width);
    info = buf;
    return true;
  }

  // ---------------------------------------------------------------- services
  void onGoObserve(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res)
  {
    if (busy_.exchange(true)) {
      res->success = false;
      res->message = "executor busy";
      return;
    }
    BusyGuard guard(busy_);
    const bool ok = openGripper() && goObserve();
    res->success = ok;
    res->message = ok ? "at observe pose" : "failed to reach observe pose";
  }

  // ---------------------------------------------------------- recovery helper
  // Put a tray we are holding back where we found it (used when the drop zone is unreachable).
  void putBack(double x, double y, double yaw)
  {
    RCLCPP_WARN(get_logger(), "Recovery: putting tray back at its pick location");
    if (planAndGo(makePose(x, y, approachZ(), yaw), "putback_above", true) &&
      cartesian({makePose(x, y, placeZ(), yaw)}, "putback_descend"))
    {
      openGripper();
      cartesian({makePose(x, y, approachZ(), yaw)}, "putback_retreat");
    } else {
      openGripper();
    }
  }

  // ------------------------------------------------------------------ action
  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    BusyGuard guard(busy_);
    const auto goal = gh->get_goal();
    auto result = std::make_shared<PickPlace::Result>();
    const auto t0 = now();

    auto stage = [&](const std::string & s) {
        auto fb = std::make_shared<PickPlace::Feedback>();
        fb->stage = s;
        gh->publish_feedback(fb);
        RCLCPP_INFO(get_logger(), "[stage] %s", s.c_str());
      };
    auto fail = [&](uint8_t code, const std::string & msg) {
        RCLCPP_ERROR(get_logger(), "Pick&place failed (%u): %s", code, msg.c_str());
        result->success = false;
        result->error_code = code;
        result->message = msg;
        gh->abort(result);
      };
    auto canceled = [&]() {
        if (!gh->is_canceling()) return false;
        mg_->stop();
        result->success = false;
        result->error_code = PickPlace::Result::CANCELED;
        result->message = "canceled";
        gh->canceled(result);
        return true;
      };

    const auto & tray = goal->tray;
    if (!tray.header.frame_id.empty() && tray.header.frame_id != frame_) {
      return fail(PickPlace::Result::INVALID_INPUT,
               "tray frame '" + tray.header.frame_id + "' != planning frame '" + frame_ + "'");
    }
    const double px = tray.pose.position.x, py = tray.pose.position.y;
    const double yaw = tray.yaw;
    const double width = tray.width > 0.03 ? tray.width : default_width_;
    const double plx = goal->place_pose.pose.position.x;
    const double ply = goal->place_pose.pose.position.y;
    const double plyaw = yawOf(goal->place_pose.pose.orientation);
    std::string info;

    // 1 ------------------------------------------------------------- open
    stage("OPEN_GRIPPER");
    if (!openGripper()) return fail(PickPlace::Result::GRIPPER_ERROR, "cannot open gripper");
    if (canceled()) return;

    // 2 ---------------------------------------------------------- pre-grasp
    stage("MOVE_TO_PREGRASP");
    double yaw_sel = yaw;
    bool ok = false;
    for (double c : {yaw, yaw + kPi}) {
      if (planAndGo(makePose(px, py, approachZ(), c), "pregrasp")) {
        yaw_sel = c;
        ok = true;
        break;
      }
    }
    if (!ok) {
      goObserve();
      return fail(PickPlace::Result::PLANNING_FAILED, "no feasible pre-grasp motion");
    }
    if (canceled()) return;

    // 3 -------------------------------------------------------------- descend
    stage("DESCEND");
    if (!cartesian({makePose(px, py, graspZ(), yaw_sel)}, "descend")) {
      goObserve();
      return fail(PickPlace::Result::EXECUTION_FAILED, "Cartesian descent to grasp pose failed");
    }

    // 4 ---------------------------------------------------------------- grasp
    stage("GRASP");
    closeGripper();
    if (!graspHeld(width, info)) {
      openGripper();
      cartesian({makePose(px, py, approachZ(), yaw_sel)}, "retreat_after_failed_grasp");
      goObserve();
      return fail(PickPlace::Result::GRASP_FAILED, info);
    }
    RCLCPP_INFO(get_logger(), "%s", info.c_str());

    // 5 ------------------------------------------------------------------ lift
    stage("LIFT");
    if (!cartesian({makePose(px, py, clearanceZ(), yaw_sel)}, "lift")) {
      openGripper();
      goObserve();
      return fail(PickPlace::Result::EXECUTION_FAILED, "lift failed (tray released)");
    }
    if (!graspHeld(width, info)) {
      openGripper();
      goObserve();
      return fail(PickPlace::Result::TRAY_DROPPED, "tray lost during lift: " + info);
    }
    if (canceled()) {
      putBack(px, py, yaw_sel);
      goObserve();
      return;
    }

    // 6 ---------------------------------------------------------------- transit
    stage("TRANSIT_TO_DROP");
    std::vector<double> cands = {plyaw, plyaw + kPi};
    std::sort(cands.begin(), cands.end(), [&](double a, double b) {
        return std::fabs(wrapAngle(a - yaw_sel)) < std::fabs(wrapAngle(b - yaw_sel));
      });
    double place_yaw = plyaw;
    ok = false;
    for (double c : cands) {
      if (planAndGo(makePose(plx, ply, clearanceZ(), c), "above_drop", true)) {
        place_yaw = c;
        ok = true;
        break;
      }
    }
    if (!ok) {
      putBack(px, py, yaw_sel);
      goObserve();
      return fail(PickPlace::Result::PLANNING_FAILED, "drop zone unreachable; tray put back");
    }
    if (!graspHeld(width, info)) {
      openGripper();
      goObserve();
      return fail(PickPlace::Result::TRAY_DROPPED, "tray lost in transit: " + info);
    }

    // 7 ---------------------------------------------------------------- place
    stage("PLACE");
    if (!cartesian({makePose(plx, ply, placeZ(), place_yaw)}, "place_descend")) {
      putBack(px, py, yaw_sel);
      goObserve();
      return fail(PickPlace::Result::EXECUTION_FAILED, "place descent failed; tray put back");
    }
    stage("RELEASE");
    if (!openGripper()) return fail(PickPlace::Result::GRIPPER_ERROR, "cannot release tray");
    simSleep(0.3);
    stage("RETREAT");
    cartesian({makePose(plx, ply, approachZ(), place_yaw)}, "place_retreat");

    // 8 --------------------------------------------------------------- observe
    stage("RETURN_TO_OBSERVE");
    if (!goObserve()) RCLCPP_WARN(get_logger(), "could not return to observe pose (non fatal)");

    result->success = true;
    result->error_code = PickPlace::Result::NONE;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "pick&place done in %.1f s", (now() - t0).seconds());
    result->message = buf;
    gh->succeed(result);
  }

  // ----------------------------------------------------------------- members
  std::string group_, ee_link_, frame_, left_joint_, right_joint_, gripper_action_;
  double table_z_{0.75}, grasp_height_{0.03}, approach_height_{0.15}, clearance_height_{0.30};
  double place_margin_{0.01}, vel_scale_{0.4}, acc_scale_{0.4}, cart_vel_scale_{0.15};
  double planning_time_{5.0}, eef_step_{0.005}, min_fraction_{0.95};
  double open_q_{0.075}, close_q_{0.01}, close_time_{2.0}, settle_time_{0.6};
  double min_finger_q_{0.015}, width_tol_{0.025}, default_width_{0.10};
  double wall_cap_{300.0};
  double cart_acc_scale_{0.15}, carry_vel_{0.3}, carry_acc_{0.15};
  int planning_attempts_{5};
  std::vector<double> observe_joints_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg_;
  rclcpp_action::Client<FJT>::SharedPtr gripper_client_;
  rclcpp_action::Server<PickPlace>::SharedPtr server_;
  rclcpp::Service<Trigger>::SharedPtr observe_srv_;
  rclcpp::CallbackGroup::SharedPtr srv_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  std::mutex js_mtx_;
  std::map<std::string, double> js_;
  std::atomic<bool> busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceExecutor>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() { exec.spin(); });
  try {
    node->init();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "init failed: %s", e.what());
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
