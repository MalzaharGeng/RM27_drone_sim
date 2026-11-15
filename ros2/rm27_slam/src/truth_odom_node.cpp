// 真值回灌节点 [SIM-ONLY]：把 Gazebo 真值包装成 ORB-SLAM3 的导航输出，
// 交给与 Nano 完全相同的 Px4Bridge（Gate / 坐标对齐 / 时间戳映射 / health）。
//
// 输入 /truth/pose 由 truth_gz_node.py 提供：按实体名筛选，header.stamp 是
// Gazebo 源 sim 时间（ros_gz_bridge 的 TF 转换会丢名字和时戳，不能用）。
//
// 本节点只做三件事：
//   1. 用 /clock 的窗口最小延迟估计 sim -> 主机 epoch 的映射（与 SLAM 节点同一
//      口径，不把样本往接收时刻推）；
//   2. 用源 sim 时间差分求速度（不用回调接收时刻，不用 EMA）；
//   3. 把 ENU 位姿交给 Px4Bridge，由它完成 R_BI / NED 对齐 / 门控。
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <utility>

#include "Px4Bridge.h"

namespace
{
struct Navigation
{
  bool valid = true;
  bool inertialInitialized = true;
  bool inertialBA2 = true;
  bool velocityValid = true;
  uint64_t referenceEpoch = 1;
  Sophus::SE3f imuToWorld;
  Eigen::Vector3f velocityWorld = Eigen::Vector3f::Zero();
  Eigen::Vector3f gravityWorld = Eigen::Vector3f(0.f, 0.f, -9.81f);
};
}  // namespace

class TruthOdomNode : public rclcpp::Node
{
public:
  TruthOdomNode()
  : rclcpp::Node("truth_odom")
  {
    const std::string px4_config = declare_parameter<std::string>("px4_config", "");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/truth/pose");
    clock_topic_ = declare_parameter<std::string>("clock_topic", "/clock");
    offset_window_s_ = declare_parameter<double>("offset_window_seconds", 2.0);
    max_step_s_ = declare_parameter<double>("max_step_s", 0.5);
    if (px4_config.empty()) {
      throw std::runtime_error("参数 px4_config 未设置");
    }
    const auto cfg = ORB_SLAM3::px4::Config::Load(px4_config);
    bridge_ = std::make_unique<ORB_SLAM3::px4::Bridge>(cfg);
    RCLCPP_INFO(get_logger(), "px4_config=%s pose=%s clock=%s",
      px4_config.c_str(), pose_topic_.c_str(), clock_topic_.c_str());

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(50).best_effort(),
      std::bind(&TruthOdomNode::onPose, this, std::placeholders::_1));
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      clock_topic_, rclcpp::QoS(50),
      std::bind(&TruthOdomNode::onClock, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/truth/odom", 50);
  }

private:
  static uint64_t stampUs(const builtin_interfaces::msg::Time & t)
  {
    return static_cast<uint64_t>(t.sec) * 1000000ULL +
           static_cast<uint64_t>(t.nanosec) / 1000ULL;
  }

  void onClock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & m)
  {
    const int64_t sim_us = static_cast<int64_t>(m->clock.sec) * 1000000 +
      static_cast<int64_t>(m->clock.nanosec) / 1000;
    const int64_t now_us = static_cast<int64_t>(ORB_SLAM3::px4::SystemMicros());
    const uint64_t steady_us = ORB_SLAM3::px4::SteadyMicros();
    const int64_t candidate = now_us - sim_us;
    if (!have_offset_) {
      offset_us_ = candidate;
      have_offset_ = true;
      RCLCPP_INFO(get_logger(), "time base: initial offset=%+.3f s", offset_us_ * 1e-6);
    }
    window_.emplace_back(steady_us, candidate);
    const uint64_t win = static_cast<uint64_t>(std::max(0.2, offset_window_s_) * 1e6);
    while (!window_.empty() && steady_us - window_.front().first > win) {
      window_.pop_front();
    }
    int64_t best = candidate;
    for (const auto & s : window_) {
      best = std::min(best, s.second);
    }
    if (best > offset_us_) {
      offset_us_ += std::min<int64_t>(best - offset_us_, 20000);
    } else if (best < offset_us_ - 200000) {
      offset_us_ = best;
      RCLCPP_WARN(get_logger(), "time base re-anchored offset=%+.3f s", offset_us_ * 1e-6);
    }
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr m)
  {
    const uint64_t sim_us = stampUs(m->header.stamp);
    if (sim_us == 0) {
      return;
    }
    if (!have_offset_) {
      return;
    }
    if (last_sim_us_ && sim_us <= last_sim_us_) {
      return;  // 源时间必须单调；倒退说明时钟域变了，交给 Gate 处理
    }
    const Eigen::Vector3d p(m->pose.position.x, m->pose.position.y, m->pose.position.z);
    Eigen::Quaterniond q(m->pose.orientation.w, m->pose.orientation.x,
                         m->pose.orientation.y, m->pose.orientation.z);
    if (!std::isfinite(p.norm()) || !std::isfinite(q.norm()) || q.norm() < 1e-6) {
      return;
    }
    q.normalize();

    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    bool velocity_valid = false;
    if (last_sim_us_) {
      const double dt = double(sim_us - last_sim_us_) * 1e-6;
      if (dt > 1e-5 && dt < max_step_s_) {
        // 用源时间差分；不平滑、不用接收时刻。
        v = (p - last_p_) / dt;
        velocity_valid = true;
      }
    }
    last_sim_us_ = sim_us;
    last_p_ = p;

    const uint64_t epoch_us = sim_us + static_cast<uint64_t>(std::max<int64_t>(0, offset_us_));

    Navigation nav;
    nav.imuToWorld = Sophus::SE3f(Sophus::SO3f(q.cast<float>()), p.cast<float>());
    nav.velocityWorld = v.cast<float>();
    nav.velocityValid = velocity_valid;
    // Gazebo 世界是 ENU，重力指向 -z；Gate 用它做重力对齐。
    nav.gravityWorld = Eigen::Vector3f(0.f, 0.f, -9.81f);

    bridge_->Submit(nav, epoch_us);

    nav_msgs::msg::Odometry odom;
    odom.header = m->header;
    odom.header.frame_id = "truth_world_enu";
    odom.child_frame_id = "base_link";
    odom.pose.pose = m->pose;
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();
    odom_pub_->publish(odom);

    ++frames_;
    if (frames_ % 200 == 0) {
      RCLCPP_INFO(get_logger(),
        "truth #%lu sim=%.3f p=(%.2f,%.2f,%.2f) v=(%.2f,%.2f,%.2f) age_ms=%.1f",
        static_cast<unsigned long>(frames_), sim_us * 1e-6, p.x(), p.y(), p.z(),
        v.x(), v.y(), v.z(),
        double(ORB_SLAM3::px4::SystemMicros() - epoch_us) * 1e-3);
    }
  }

  std::string pose_topic_, clock_topic_;
  double offset_window_s_{2.0};
  double max_step_s_{0.5};
  std::unique_ptr<ORB_SLAM3::px4::Bridge> bridge_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::deque<std::pair<uint64_t, int64_t>> window_;
  bool have_offset_{false};
  int64_t offset_us_{0};
  uint64_t last_sim_us_{0};
  Eigen::Vector3d last_p_{Eigen::Vector3d::Zero()};
  uint64_t frames_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TruthOdomNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("truth_odom"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
