// RM27 仿真：双目 + IMU 的 ORB_SLAM3 节点（与实机同一算法路径）
//
// 迁移标注（与 config/px4_vio_sim.yaml 同一套约定）：
//   [SAME]     与实机同一实现，可迁移
//   [SIM-ONLY] 只存在于仿真，不可迁移到实机
//   [PORTABLE] 修的是缺陷，实机同样成立
//
// 本文件整体属于 [SIM-ONLY]：它是「Gazebo 数据源 → ORB_SLAM3」的适配层，
// 实机对应的是 stereo_inertial_realsense_D435i（librealsense 取流）。
// 其中真正共享的是它调用的 ORB_SLAM3::System 与 Px4Bridge/Px4Vio，
// 那部分改动的可迁移性见 src/Px4Vio.cc 内的 [PORTABLE] 标注。
//
// 数据链：
//   Gazebo 双目图像 ─┐
//   Gazebo IMU ──────┼─ ros_gz_bridge ─ ROS 2 ─ 本节点
//   Gazebo /clock ───┘
//                        │  ORB_SLAM3::System::IMU_STEREO
//                        │  Px4Bridge（Gate / SensorClock / health）
//                        ▼
//                 /fmu/in/vehicle_visual_odometry（DDS）
//                 /vio/navigation_health（真实门控健康，不是仿真桩）
//                 /slam/pose（仅监控/精度评估）
//
// 与实机 stereo_inertial_realsense_D435i 的差异只在“数据源”：
//   * [SIM-ONLY] 实机用 librealsense 的 global time；仿真用 gz sim time，经 /clock
//     映射到 epoch 微秒后再交给 SensorClock，保持“源时间”语义。这段换算依赖
//     RTF，RTF≠1 时 epoch 会持续偏移，属于仿真传输层伪影。
//   * [SIM-GEOMETRY] 相机内参、基线、相机-IMU 外参全部来自仿真几何
//     （RM27_Sim_IMU_Stereo.yaml），不复制实机标定。
//
// 关键边界：
//   * 不用发布时刻冒充采样时刻；图像/IMU 的时间戳来自 gz 消息头。
//   * [SIM-ONLY] 左右目按源时间配对；超过 sync_tolerance 的帧丢弃并计数。
//     实机双目由同一驱动在同一帧内输出，硬件同步，不需要这层配对。
//   * 时钟域跳变/传感器时间倒退会让 SensorClock 重置 epoch，并重置当前地图，
//     不会用旧观测继续续期。
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>
#include <utility>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

#include "System.h"
#include "ImuTypes.h"
#include "InputRecording.h"
#include "Px4Bridge.h"
#include "time_trace.hpp"
#include <rclcpp/message_info.hpp>

// [WORKPKG-TRACE] image_group_ is MutuallyExclusive, so onImage ->
// processPair is serial on one executor thread. A thread_local seq keeps
// the image/IMU/clock groups from clobbering each other without changing
// any queue type or adding a per-frame heavy lock.
namespace { thread_local uint64_t g_pending_trace_seq = 0; }

namespace
{
cv::Mat imageToMat(const sensor_msgs::msg::Image & msg)
{
  int type;
  if (msg.encoding == "mono8") {
    type = CV_8UC1;
  } else if (msg.encoding == "rgb8" || msg.encoding == "bgr8") {
    type = CV_8UC3;
  } else if (msg.encoding == "rgba8" || msg.encoding == "bgra8") {
    type = CV_8UC4;
  } else {
    throw std::runtime_error("unsupported encoding: " + msg.encoding);
  }

  cv::Mat wrapped(msg.height, msg.width, type,
                  const_cast<unsigned char *>(msg.data.data()), msg.step);
  cv::Mat out = wrapped.clone();
  if (type == CV_8UC4) {
    cv::cvtColor(out, out, cv::COLOR_BGRA2BGR);
  }
  return out;
}

const char * stateName(int s)
{
  switch (s) {
    case -1: return "SYSTEM_NOT_READY";
    case 0:  return "NO_IMAGES_YET";
    case 1:  return "NOT_INITIALIZED";
    case 2:  return "OK";
    case 3:  return "RECENTLY_LOST";
    case 4:  return "LOST";
    case 5:  return "OK_KLT";
    default: return "UNKNOWN";
  }
}

double stampSeconds(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}
}  // namespace

class StereoInertialSim : public rclcpp::Node
{
public:
  StereoInertialSim()
  : Node("stereo_inertial_sim_node")
  {
    const char * home_env = std::getenv("HOME");
    const std::string home = home_env ? home_env : "";

    vocab_file_ = declare_parameter<std::string>(
      "vocab_file", home + "/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    settings_file_ = declare_parameter<std::string>(
      "settings_file", home + "/ORB_SLAM3/config/RM27_Sim_IMU_Stereo.yaml");
    px4_config_ = declare_parameter<std::string>(
      "px4_config", home + "/ORB_SLAM3/config/px4_vio_sim.yaml");

    left_topic_ = declare_parameter<std::string>(
      "left_topic", "/world/rm27_world/model/x500_mono_cam_0/link/cam_left_link/sensor/cam_left/image");
    right_topic_ = declare_parameter<std::string>(
      "right_topic", "/world/rm27_world/model/x500_mono_cam_0/link/cam_right_link/sensor/cam_right/image");
    imu_topic_ = declare_parameter<std::string>(
      "imu_topic", "/world/rm27_world/model/x500_mono_cam_0/link/base_link/sensor/imu_sensor/imu");
    clock_topic_ = declare_parameter<std::string>("clock_topic", "/clock");

    pose_topic_ = declare_parameter<std::string>("pose_topic", "/slam/pose");
    state_topic_ = declare_parameter<std::string>("state_topic", "/slam/tracking_state");
    use_viewer_ = declare_parameter<bool>("use_viewer", false);
    sync_tol_ = declare_parameter<double>("sync_tolerance", 0.005);
    max_imu_buffer_ = declare_parameter<int>("max_imu_buffer", 5000);
    offset_window_s_ = declare_parameter<double>("offset_window_seconds", 2.0);
    require_imu_ = declare_parameter<bool>("require_imu", true);
    record_dir_ = declare_parameter<std::string>("record_dir", "");
    latency_csv_ = declare_parameter<std::string>("latency_csv", "");
    trace_csv_ = declare_parameter<std::string>("trace_csv", "");
    // [PHASE7-DIAG] Runtime switch for the phase5 observation path.
    // Default ON is behaviour-equivalent to the phase5 binary. OFF skips the
    // whole added observation path: no 21-column CSV, no per-frame
    // GetNavigationState for observation, no cached mapping values.
    // Functional state queries (Bridge::Submit / Gate) are NEVER skipped.
    {
      const char* obs_env = std::getenv("SIM_LATENCY_OBSERVATION");
      const std::string obs_val = obs_env ? obs_env : "on";
      obs_enabled_ = !(obs_val == "off" || obs_val == "OFF" || obs_val == "0");
      RCLCPP_INFO(get_logger(), "[PHASE7] SIM_LATENCY_OBSERVATION=%s",
                  obs_enabled_ ? "on" : "off");
    }
    // [SIM-ONLY] 上一版为掩盖漂移帧加了 30 m/s^2 的驱动层速度预检；它改的是
    // 「交给 Gate 的样本是否有效」，与实机路径不同。默认关闭，需要时显式打开。
    sim_nav_guard_ = declare_parameter<bool>("sim_nav_guard", false);

    // [NP01-FIX15] 启动生命周期闸门。off = 与改造前完全一致。
    init_gate_ = declare_parameter<std::string>("init_gate", "off");
    land_topic_ = declare_parameter<std::string>(
      "land_topic", "/fmu/out/vehicle_land_detected");
    init_gate_airborne_hold_ = declare_parameter<double>("init_gate_airborne_hold", 0.50);
    init_gate_health_window_ = declare_parameter<double>("init_gate_health_window", 1.00);
    init_gate_min_img_hz_ = declare_parameter<double>("init_gate_min_img_hz", 15.0);
    init_gate_max_first_age_ms_ =
      declare_parameter<double>("init_gate_max_first_age_ms", 250.0);
    if (init_gate_ != "off" && init_gate_ != "airborne") {
      throw std::runtime_error("init_gate must be 'off' or 'airborne', got: " + init_gate_);
    }

    RCLCPP_INFO(get_logger(), "vocab   : %s", vocab_file_.c_str());
    RCLCPP_INFO(get_logger(), "settings: %s", settings_file_.c_str());
    RCLCPP_INFO(get_logger(), "px4 cfg : %s", px4_config_.c_str());

    // 与实机示例一致：EV 发布要求 loopClosing: 0，避免控制期间回环/GBA 改写参考系。
    {
      cv::FileStorage fs(settings_file_, cv::FileStorage::READ);
      if (!fs.isOpened()) {
        throw std::runtime_error("Cannot read ORB settings: " + settings_file_);
      }
      const auto lc = fs["loopClosing"];
      if (!lc.empty() && static_cast<int>(lc) != 0) {
        throw std::runtime_error(
          "Px4Bridge EV publication requires loopClosing: 0 in " + settings_file_);
      }
    }

    const auto px4_cfg = ORB_SLAM3::px4::Config::Load(px4_config_);
    bridge_ = std::make_unique<ORB_SLAM3::px4::Bridge>(px4_cfg);

    RCLCPP_INFO(get_logger(), "loading ORB_SLAM3 vocabulary (this takes a while)...");
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocab_file_, settings_file_, ORB_SLAM3::System::IMU_STEREO, use_viewer_);
    if (!record_dir_.empty()) {
      recording_ = std::make_shared<ORB_SLAM3::InputRecording>(
        record_dir_, settings_file_);
      slam_->SetInputRecording(recording_);
    }
    if (obs_enabled_ && !latency_csv_.empty()) {
      latency_.open(latency_csv_);
      if (!latency_) {
        throw std::runtime_error("Cannot open latency csv: " + latency_csv_);
      }
      // [PHASE5-OBS] Observation-only extension. The first 8 columns are
      // byte-identical to the previous format so existing tooling still works.
      // Clock domains are explicit in the names: *_mono = SteadyMicros (host
      // monotonic), *_wall = SystemMicros (host epoch), sim = Gazebo sim time.
      // sample_us = sim_us + map_offset_us is a HOST-EPOCH-domain estimate.
      // submit_* is the ENQUEUE boundary, NOT the publish boundary.
      latency_ << "pair,sim_us,rx_mono_us,rx_wall_us,track_start_us,track_end_us,"
                  "imu_count,tracking_state,"
                  "obs_status,obs_reason,src_sim_us,sample_us,map_offset_us,"
                  "map_offset_valid,clock_epoch,reference_epoch,submit_seq,"
                  "submit_wall_us,submit_mono_us,time_valid,offset_jumps\n";
    }
    if (!trace_csv_.empty()) {
      trace_ = std::make_unique<TimeTrace>(trace_csv_);
      RCLCPP_INFO(get_logger(), "[WORKPKG-TRACE] time trace: %s",
                  trace_csv_.c_str());
    }
    RCLCPP_INFO(get_logger(), "ORB_SLAM3 IMU_STEREO ready");

    image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // IMU/时钟必须保持消息顺序：Reentrant 组会让两条相邻消息并发处理，
    // 后一条先拿到锁时时间戳会「倒退 4ms」，SensorClock 会把整个 epoch 重置，
    // 已发布过 EV 的飞行中还会触发 requires_disarm 的锁存故障。
    imu_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    clock_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).best_effort();
    auto clock_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    rclcpp::SubscriptionOptions img_opt;
    img_opt.callback_group = image_group_;
    rclcpp::SubscriptionOptions imu_opt;
    imu_opt.callback_group = imu_group_;
    rclcpp::SubscriptionOptions clock_opt;
    clock_opt.callback_group = clock_group_;

    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      left_topic_, img_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m,
             const rclcpp::MessageInfo &info) {onImage(true, m, info);}, img_opt);
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      right_topic_, img_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m,
             const rclcpp::MessageInfo &info) {onImage(false, m, info);}, img_opt);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, imu_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr m,
             const rclcpp::MessageInfo &info) {onImu(m, info);}, imu_opt);
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      clock_topic_, clock_qos,
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr m,
             const rclcpp::MessageInfo &info) {onClock(m, info);}, clock_opt);

    // [NP01-FIX15] 启动生命周期：把「传感器/时钟就绪」与「Tracking 开始初始化」分开。
    // 离地判据只能来自飞控自身的状态估计（PX4 land detector：气压 + IMU + 推力），
    // 不读真值、不读 VIO 输出，因此不存在「要先有 VIO ready 才能产生初始化运动」的循环。
    // init_gate=off 时行为与改造前完全一致（回归测试与旧结果不受影响）。
    if (init_gate_ == "airborne") {
      auto land_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
      rclcpp::SubscriptionOptions land_opt;
      land_opt.callback_group = clock_group_;
      land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
        land_topic_, land_qos,
        [this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr m) {
          onLandDetected(m);
        }, land_opt);
      RCLCPP_INFO(get_logger(),
        "init_gate=airborne | land_topic=%s hold=%.2fs health_window=%.2fs "
        "min_img_hz=%.1f max_first_age=%.0fms",
        land_topic_.c_str(), init_gate_airborne_hold_, init_gate_health_window_,
        init_gate_min_img_hz_, init_gate_max_first_age_ms_);
    } else {
      RCLCPP_INFO(get_logger(), "init_gate=off (legacy behaviour: feed frames from start)");
    }

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32>(state_topic_, 10);

    status_timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {report();});
    perf_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {perfReport();});

    RCLCPP_INFO(get_logger(), "left  : %s", left_topic_.c_str());
    RCLCPP_INFO(get_logger(), "right : %s", right_topic_.c_str());
    RCLCPP_INFO(get_logger(), "imu   : %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "clock : %s", clock_topic_.c_str());
    RCLCPP_INFO(get_logger(), "pose  : %s", pose_topic_.c_str());
    RCLCPP_INFO(get_logger(),
      "sync_tolerance=%.3f s | require_imu=%d | offset_window=%.1f s",
      sync_tol_, static_cast<int>(require_imu_), offset_window_s_);
  }

  ~StereoInertialSim() override
  {
    // [NP-01 FIX13] Evidence durability. Observed in
    // np01_fix12_func1m_20261115_165203: when ORB-SLAM3 VIBA2 never completed,
    // System::SaveTrajectoryTUM() segfaulted during shutdown. With the old
    // ordering (bridge_.reset() LAST) that crash also skipped
    // Bridge::~Impl -> GateSnapshotTrace::Flush(true), plus latency/trace
    // flushes, leaving gate_snapshot.csv at 1024 of ~1893 submitted samples.
    //
    // Shutdown() must stay FIRST: it stops the Tracking threads that call
    // Bridge::Submit. Only then is it safe to destroy the Bridge.
    // This is an EVIDENCE-ONLY reorder: no Gate/LIMITS/age/fusion/acceptance
    // threshold and no control behaviour is changed.
    if (slam_) {
      slam_->Shutdown();
    }
    bridge_.reset();                 // forces FIX11 Gate snapshot Flush(true)
    if (latency_.is_open()) {
      latency_.flush();
      latency_.close();
    }
    if (trace_) {
      trace_->Flush(true);
      std::string counters;
      trace_->DumpCounters(&counters);
      RCLCPP_INFO(get_logger(), "[WORKPKG-TRACE] %s", counters.c_str());
    }
    if (recording_ && !recording_->Close()) {
      RCLCPP_ERROR(get_logger(), "input recording did not finalize cleanly");
    }
    if (slam_) {
      // Upstream ORB-SLAM3 can abort here if IMU initialisation never
      // completed. All run evidence is ALREADY flushed at this point.
      RCLCPP_WARN(get_logger(),
        "[NP01_FIX13] exporting TUM trajectory; upstream SaveTrajectoryTUM "
        "may abort when VIBA2 never completed. Run evidence already flushed.");
      slam_->SaveTrajectoryTUM("/tmp/slam_imu_stereo_trajectory_tum.txt");
      RCLCPP_INFO(get_logger(), "trajectory saved: /tmp/slam_imu_stereo_trajectory_tum.txt");
    }
  }

private:
  struct ImuSample
  {
    double t;
    float ax, ay, az, gx, gy, gz;
  };

  // ---- 时间基准：gz sim time -> epoch 微秒 ---------------------------------
  // Gazebo 消息头是 sim time；Px4Bridge/Gate 用 SystemMicros（epoch）判断年龄，
  // 并且要求与 PX4 local position 的 epoch 时戳相差 <150ms。
  // /clock 的到达时刻给出 (epoch - sim) 的样本，取短窗口内中位数消掉单个
  // DDS 延迟尖峰。窗口不能太长：仿真 RTF<1 时 offset 会随 sim 落后于
  // wall 而持续增大，长窗口会把这段漂移当成固定偏差（2s 窗口在 RTF 0.9 下
  // 就有约 200ms 偏差，足以触发 Gate 的 150ms 检查失败）。
  // 这个映射只改变时间域，不改变采样顺序。
  int64_t simToEpochUsLocked(int64_t sim_us)
  {
    if (!have_offset_) {
      offset_us_ = static_cast<int64_t>(ORB_SLAM3::px4::SystemMicros()) - sim_us;
      have_offset_ = true;
      RCLCPP_INFO(get_logger(), "time base: initial offset=%+.3f s", offset_us_ * 1e-6);
    }
    return sim_us + offset_us_;
  }

  void onClock(const rosgraph_msgs::msg::Clock::ConstSharedPtr & m,
               const rclcpp::MessageInfo &info)
  {
    const int64_t sim_us = static_cast<int64_t>(m->clock.sec) * 1000000 +
      static_cast<int64_t>(m->clock.nanosec) / 1000;
    const int64_t now_us = static_cast<int64_t>(ORB_SLAM3::px4::SystemMicros());
    const uint64_t steady_us = ORB_SLAM3::px4::SteadyMicros();
    g_pending_trace_seq = TraceAdd(3, sim_us, info);
    if (trace_) trace_->Mark(g_pending_trace_seq, 1, 0, now_us, steady_us,
                            offset_us_, 0);

    std::lock_guard<std::mutex> lock(clock_mutex_);
    // epoch_us = sim_us + offset_us 是「源采样时刻在主机 epoch 时钟上的估计」。
    // 每次 /clock 回调给出一次观测 candidate = 收到时刻 - 源时刻，它等于真实
    // offset 加上本次 /clock 的传输延迟（>=0）。取短窗口最小值即可消掉传输
    // 延迟尖峰，得到当前时刻最紧的上界——不把图像/IMU 的样本往接收时刻推，
    // 也不做 P90/EMA 之类的“把 age 压下去”的平滑。
    const int64_t candidate = now_us - sim_us;
    if (!have_offset_) {
      offset_us_ = candidate;
      have_offset_ = true;
      RCLCPP_INFO(get_logger(), "time base: initial offset=%+.3f s", offset_us_ * 1e-6);
    }
    offset_window_.emplace_back(steady_us, candidate);
    const uint64_t window_us =
      static_cast<uint64_t>(std::max(0.2, offset_window_s_) * 1e6);
    while (!offset_window_.empty() && steady_us - offset_window_.front().first > window_us) {
      offset_window_.pop_front();
    }
    int64_t window_min = candidate;
    for (const auto & s : offset_window_) {
      window_min = std::min(window_min, s.second);
    }
    // sim 落后于 wall（RTF<1）时 offset 必须单调上移；反向只在明显回退
    // （时钟重建/世界重载）时吸附，避免把单次延迟尖峰写进映射。
    if (window_min > offset_us_) {
      const int64_t step = std::min<int64_t>(window_min - offset_us_, 20000);
      offset_us_ += step;
    } else if (window_min < offset_us_ - 200000) {
      offset_us_ = window_min;
      ++time_jumps_;
      RCLCPP_WARN(get_logger(), "time base re-anchored: offset=%+.3f s jumps=%d",
        offset_us_ * 1e-6, time_jumps_);
    }
    last_clock_sim_us_ = sim_us;
    last_clock_rx_us_ = steady_us;
  }

  // [WORKPKG-TRACE] first line of each callback; rmw timestamp is host
  // epoch ns -> us. 0 means "RMW did not populate it", never a real 0.
  uint64_t TraceAdd(int stream, int64_t sim_us,
                    const rclcpp::MessageInfo &info) {
    const uint64_t w = ORB_SLAM3::px4::SystemMicros();
    const uint64_t mo = ORB_SLAM3::px4::SteadyMicros();
    const int64_t raw = info.get_rmw_message_info().received_timestamp;
    const uint64_t rmw = raw > 0 ? static_cast<uint64_t>(raw / 1000) : 0;
    return trace_ ? trace_->Add(stream, static_cast<uint64_t>(sim_us),
                               rmw, w, mo) : 0;
  }

  bool acceptLocked(unsigned stream, double t_sim)
  {
    const int64_t sim_us = static_cast<int64_t>(std::llround(t_sim * 1e6));
    if (sim_us <= 0) {
      return false;
    }
    const int64_t epoch_us = simToEpochUsLocked(sim_us);
    const int64_t wall_us = static_cast<int64_t>(ORB_SLAM3::px4::SystemMicros());
    const uint64_t mono_us = ORB_SLAM3::px4::SteadyMicros();
    ORB_SLAM3::px4::SensorClockReject why;
    const bool ok = sensor_clock_.Accept(
      stream, true, static_cast<double>(epoch_us) * 1e-3, wall_us, mono_us, &why);
    if (ok) {
      last_stream_ts_[stream] = t_sim;
      last_stream_mono_[stream] = mono_us;
    } else {
      // [NP01-FIX15] 打印真实的布尔原因与触发停更的流。
      // gap 只是「本路」的间隔；本路 gap 很小并不代表别的流没停更——
      // stalled 命中时 stalledStream 才是根因，且它会把拒绝升级为 domainReset。
      const double age_ms = static_cast<double>(wall_us - epoch_us) * 1e-3;
      const double dt_ms = last_stream_ts_[stream] > 0
        ? (t_sim - last_stream_ts_[stream]) * 1e3 : -1.0;
      const double mono_gap_ms = last_stream_mono_[stream]
        ? static_cast<double>(mono_us - last_stream_mono_[stream]) * 1e-3 : -1.0;
      ++n_clock_reject_reason_[reasonIndex(why)];
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "clock reject stream=%u t=%.3f age=%.1fms dt=%.1fms gap=%.1fms offset=%.3fs "
        "why{domainFault=%d staleSample=%d hostArtefact=%d stalled=%d stalledStream=%d "
        "srcFault=%d domainReset=%d} epoch=%llu",
        stream, t_sim, age_ms, dt_ms, mono_gap_ms, offset_us_ * 1e-6,
        int(why.domainFault), int(why.staleSample), int(why.hostArtefact),
        int(why.stalled), why.stalledStream, int(why.srcFault), int(why.domainReset),
        static_cast<unsigned long long>(sensor_clock_.Epoch()));
    }
    return ok;
  }

  // [NP01-FIX15] 把拒绝原因归一到一个互斥桶，便于在 [stat] 行做聚合。
  // 顺序按"根因优先级"：域故障 > 停更 > 源时间故障 > 陈旧 > 主机假象。
  static int reasonIndex(const ORB_SLAM3::px4::SensorClockReject & w)
  {
    if (w.domainFault) return 0;
    if (w.stalled) return 1;
    if (w.srcFault) return 2;
    if (w.staleSample) return 3;
    if (w.hostArtefact) return 4;
    return 5;
  }

  // [FIX3] 按 ORB_SLAM3 的区间算术判定覆盖，不引入任何数量门限。
  //   has_front : 存在样本 t >= prev_ts - mImuPer（区间起点可插值）
  //   has_back  : 存在样本 t >= cur_ts  - mImuPer（末端边界，交出去后由
  //               PreintegrateIMU 保留给下一帧）
  //   n_window  : 落在 [prev_ts - mImuPer, cur_ts - mImuPer) 的可消费样本数
  // 只有 has_front && has_back && n_window > 0 时 n = size-1 >= 1。
  // 首帧（prev_ts < 0）没有 prevFrame，ORB 不做预积分；这里仍要求
  // 末端边界 + 至少 1 个可消费样本，保证下一帧有可用的区间起点。
  bool imuCoverageLocked(double prev_ts, double cur_ts,
                         bool & has_front, bool & has_back, size_t & n_window) const
  {
    constexpr double kImuPeriod = 1.0 / 250.0;  // settings: IMU.Frequency = 250
    constexpr double kEps = 1e-6;
    has_front = (prev_ts < 0.0);
    has_back = false;
    n_window = 0;
    for (const auto & s : imu_buf_) {
      if (s.t > cur_ts + kEps) {
        break;
      }
      if (prev_ts >= 0.0 && s.t < prev_ts - kImuPeriod - kEps) {
        continue;  // 早于区间起点的样本：ORB 会丢弃，不参与覆盖
      }
      has_front = true;
      if (s.t >= cur_ts - kImuPeriod - kEps) {
        has_back = true;
      } else {
        ++n_window;
      }
    }
    return has_front && has_back && n_window > 0;
  }

  // [FIX3] 有界等待：只延迟提交「这一对图像」，等它的 IMU 区间覆盖成立。
  // 最长等待 = 1 个标称双目周期（30 Hz → 33.333 ms）的**源时间**，
  // 依据是「超过一个帧周期仍未到齐，说明本帧输入已错过，继续等只会把
  // 延迟传给下一帧」。等待以 /clock 的源时间为基准，因此 RTF 变化不改变语义；
  // 另有 500 ms 主机时间活性保护，防止 /clock 停更时卡死。
  // 超时后由调用方跳过该图像对，imu_buf_ 一个样本都不消费。
  bool waitImuCoverage(double prev_ts, double cur_ts)
  {
    constexpr double kNominalFramePeriodS = 1.0 / 30.0;
    constexpr uint64_t kLivenessGuardUs = 500000;
    const uint64_t wall_start = ORB_SLAM3::px4::SteadyMicros();
    int64_t sim_start = -1;
    for (;;) {
      bool has_front = false, has_back = false;
      size_t n_window = 0;
      bool covered = false;
      {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        covered = imuCoverageLocked(prev_ts, cur_ts, has_front, has_back, n_window);
        cov_front_ = has_front;
        cov_back_ = has_back;
        cov_n_ = n_window;
      }
      if (covered) {
        return true;
      }
      int64_t sim_now = 0;
      {
        std::lock_guard<std::mutex> lock(clock_mutex_);
        sim_now = last_clock_sim_us_;
      }
      if (sim_start < 0) {
        sim_start = sim_now;
      }
      const bool sim_expired =
        sim_now > sim_start &&
        static_cast<double>(sim_now - sim_start) * 1e-6 >= kNominalFramePeriodS;
      const bool wall_expired =
        ORB_SLAM3::px4::SteadyMicros() - wall_start > kLivenessGuardUs;
      if (sim_expired || wall_expired) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }

  void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr & m,
             const rclcpp::MessageInfo &info)
  {
    ++n_imu_;
    if (init_gate_ == "airborne") {
      // [NP01-FIX15] 只记录到达节奏用于健康窗口；不影响 IMU 的接受/缓冲逻辑。
      const uint64_t now_us = ORB_SLAM3::px4::SteadyMicros();
      std::lock_guard<std::mutex> glock(gate_mutex_);
      imu_rx_us_.push_back(now_us);
      if (imu_rx_us_.size() > 4000) {
        imu_rx_us_.pop_front();
      }
    }
    const int64_t sim_us =
      static_cast<int64_t>(m->header.stamp.sec) * 1000000 +
      static_cast<int64_t>(m->header.stamp.nanosec) / 1000;
    g_pending_trace_seq = TraceAdd(2, sim_us, info);
    double t = stampSeconds(m->header.stamp);
    if (t <= 0.0) {
      ++n_zero_stamp_;
      return;
    }

    bool accepted = false;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      // 防御性去重：DDS/执行器仍可能把一条旧 IMU 排到新 IMU 之后。
      // 小幅倒退（<50ms）是乱序，直接丢弃该条；大幅倒退才交给 SensorClock
      // 判定为时钟域跳变（那会重置 epoch 并要求重新建图）。
      if (last_imu_t_ > 0.0 && t < last_imu_t_ - 1e-9) {
        ++n_imu_reorder_;
        if (last_imu_t_ - t < 0.05) {
          return;
        }
      }
      // gz 的 IMU 是 gyro+accel 同帧；两个 stream 用同一源时刻登记。
      const bool gyro_ok = acceptLocked(2, t);
      const bool accel_ok = acceptLocked(3, t);
      accepted = gyro_ok && accel_ok;
      if (accepted) {
        last_imu_t_ = t;
      } else {
        ++n_clock_reject_;
      }
    }

    if (trace_) trace_->Mark(g_pending_trace_seq, accepted ? 1 : 2,
                            accepted ? 0 : 1,
                            ORB_SLAM3::px4::SystemMicros(),
                            ORB_SLAM3::px4::SteadyMicros(),
                            offset_us_, 0);

    ImuSample s;
    s.t = t;
    s.ax = static_cast<float>(m->linear_acceleration.x);
    s.ay = static_cast<float>(m->linear_acceleration.y);
    s.az = static_cast<float>(m->linear_acceleration.z);
    s.gx = static_cast<float>(m->angular_velocity.x);
    s.gy = static_cast<float>(m->angular_velocity.y);
    s.gz = static_cast<float>(m->angular_velocity.z);

    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (!accepted) {
      // 时钟域被重置：旧 IMU 与新图像不再属于同一时间域，必须丢弃。
      imu_buf_.clear();
      return;
    }
    imu_buf_.push_back(s);
    while (static_cast<int>(imu_buf_.size()) > max_imu_buffer_) {
      imu_buf_.pop_front();
      ++n_imu_dropped_;
    }
  }

  // ---- [NP01-FIX15] 启动生命周期闸门 ---------------------------------------
  // 分层：
  //   L0 传感器层（节点启动即工作）：订阅、/clock 时间映射、IMU 有界缓存、
  //      输入健康统计。与是否开始 Tracking 无关。
  //   L1 初始化闸门（本函数）：由「明确事件」打开——飞控判定的离地状态持续
  //      一段时间，且输入健康窗口达标。判据来自算法输入契约与实际运动阶段，
  //      不读真值，也不要求先有 VIO 输出。
  //   L2 喂帧：闸门打开前，配好的帧一律丢弃并计数，绝不喂给 SLAM；IMU 缓存
  //      继续按 max_imu_buffer 有界增长，保证闸门打开时第一帧的 IMU 区间完整。
  void onLandDetected(const px4_msgs::msg::VehicleLandDetected::ConstSharedPtr & m)
  {
    const uint64_t now_us = ORB_SLAM3::px4::SteadyMicros();
    std::lock_guard<std::mutex> lock(gate_mutex_);
    land_landed_ = m->landed;
    land_maybe_landed_ = m->maybe_landed;
    land_ground_contact_ = m->ground_contact;
    land_last_mono_us_ = now_us;
    const bool airborne = !m->landed && !m->maybe_landed && !m->ground_contact;
    if (airborne) {
      if (airborne_since_us_ == 0) {
        airborne_since_us_ = now_us;
      }
    } else {
      airborne_since_us_ = 0;
    }
  }

  // 返回 true 表示本批帧可以喂给 SLAM。副作用：维护健康窗口、记录闸门开合。
  bool initGateAllowsFrames(size_t npairs, double pair_ts)
  {
    if (init_gate_ != "airborne") {
      return true;  // 旧行为
    }
    const uint64_t now_us = ORB_SLAM3::px4::SteadyMicros();
    bool open_now = false;
    {
      std::lock_guard<std::mutex> glock(gate_mutex_);
      open_now = gate_open_;
    }
    bool clock_ready = false;
    uint64_t epoch = 0;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      if (!open_now) {
        // [NP01-FIX15b] 闸门关闭时也必须让时钟域看到图像流。否则 streams_[0]/[1]
        // 永不登记，Ready() 恒为假，而闸门又要求 Ready()，形成自锁——
        // 第 11 轮实测：gate=CLOSED(landed) 恒定、clock ready=0 恒定、img 已到 545 对。
        // 这里只登记时间戳与时钟域，不把任何帧交给 SLAM，也不消费 IMU。
        acceptLocked(0, pair_ts);
        acceptLocked(1, pair_ts);
      }
      clock_ready = sensor_clock_.Ready();
      epoch = sensor_clock_.Epoch();
    }

    std::lock_guard<std::mutex> lock(gate_mutex_);
    if (gate_open_) {
      return true;
    }

    const uint64_t window_us =
      static_cast<uint64_t>(std::max(0.2, init_gate_health_window_) * 1e6);
    auto prune = [&](std::deque<uint64_t> & d) {
        while (!d.empty() && now_us - d.front() > window_us) {
          d.pop_front();
        }
      };
    prune(left_rx_us_);
    prune(right_rx_us_);
    prune(imu_rx_us_);
    const double win_s = std::max(0.2, init_gate_health_window_);
    const double left_hz = static_cast<double>(left_rx_us_.size()) / win_s;
    const double right_hz = static_cast<double>(right_rx_us_.size()) / win_s;
    const double imu_hz = static_cast<double>(imu_rx_us_.size()) / win_s;

    // (1) 离地事件持续
    if (airborne_since_us_ == 0) {
      gate_block_reason_ = "landed";
      return false;
    }
    if (static_cast<double>(now_us - airborne_since_us_) * 1e-6 <
      init_gate_airborne_hold_)
    {
      gate_block_reason_ = "airborne_hold";
      return false;
    }
    // (2) 飞控状态本身必须新鲜（不能靠一条旧消息永远保持"离地"）
    if (land_last_mono_us_ == 0 ||
      static_cast<double>(now_us - land_last_mono_us_) * 1e-6 > 0.5)
    {
      gate_block_reason_ = "land_state_stale";
      return false;
    }
    // (3) 输入健康：时钟就绪 + 双目速率 + 双目平衡 + IMU 速率
    if (!clock_ready) {
      gate_block_reason_ = "clock_not_ready";
      return false;
    }
    if (left_hz < init_gate_min_img_hz_ || right_hz < init_gate_min_img_hz_) {
      gate_block_reason_ = "img_rate_low";
      return false;
    }
    const double hi = std::max(left_hz, right_hz);
    const double lo = std::min(left_hz, right_hz);
    if (hi > 0.0 && (hi - lo) / hi > 0.20) {
      gate_block_reason_ = "img_unbalanced";
      return false;
    }
    if (imu_hz < 200.0) {
      gate_block_reason_ = "imu_rate_low";
      return false;
    }
    // (4) 打开。丢弃已配好的历史帧，从下一对新鲜帧开始，绝不回放积压。
    gate_open_ = true;
    gate_opened_us_ = now_us;
    gate_open_epoch_ = epoch;
    gate_block_reason_.clear();
    RCLCPP_INFO(get_logger(),
      "[INIT_GATE] OPEN after %.2fs airborne | left=%.1fHz right=%.1fHz imu=%.1fHz "
      "clock_ready=%d epoch=%llu | dropped_before_open=%d pairs, %d left / %d right imgs",
      static_cast<double>(now_us - airborne_since_us_) * 1e-6, left_hz, right_hz, imu_hz,
      static_cast<int>(clock_ready), static_cast<unsigned long long>(epoch),
      n_gate_skip_, n_gate_skip_img_l_, n_gate_skip_img_r_);
    (void)npairs;
    // 打开的这一对**不喂**：它的时间戳刚刚在闸门里登记过，再交给 processPair
    // 会被 SensorClock 判为重复源时间（srcFault）并重置时钟域。下一对才是第一帧，
    // 代价只是一帧（33 ms），换来的是零重复登记。
    return false;
  }

  void onImage(bool left, const sensor_msgs::msg::Image::ConstSharedPtr & m,
               const rclcpp::MessageInfo &info)
  {
    const int64_t img_sim_us =
      static_cast<int64_t>(m->header.stamp.sec) * 1000000 +
      static_cast<int64_t>(m->header.stamp.nanosec) / 1000;
    g_pending_trace_seq = TraceAdd(left ? 0 : 1, img_sim_us, info);
    if (init_gate_ == "airborne") {
      const uint64_t now_us = ORB_SLAM3::px4::SteadyMicros();
      std::lock_guard<std::mutex> glock(gate_mutex_);
      (left ? left_rx_us_ : right_rx_us_).push_back(now_us);
      if (!gate_open_) {
        (left ? ++n_gate_skip_img_l_ : ++n_gate_skip_img_r_);
      }
    }
    std::vector<std::pair<sensor_msgs::msg::Image::ConstSharedPtr,
                          sensor_msgs::msg::Image::ConstSharedPtr>> pairs;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      auto & q = left ? left_q_ : right_q_;
      if (left) {
        ++n_left_;
      } else {
        ++n_right_;
      }
      q.push_back(m);
      // 上限防止某一侧长期缺帧时队列无限增长
      while (q.size() > 5) {
        q.pop_front();
        ++n_skew_;
      }

      // 按源时间配对，而不是「任一侧到达就消费最新一对」。桥接后的左右目
      // 到达顺序可能差一帧；旧做法会把 t 与 t-33ms 配成一对再因超容差丢弃。
      while (!left_q_.empty() && !right_q_.empty()) {
        const double tl = stampSeconds(left_q_.front()->header.stamp);
        const double tr = stampSeconds(right_q_.front()->header.stamp);
        if (std::abs(tl - tr) <= sync_tol_) {
          pairs.emplace_back(left_q_.front(), right_q_.front());
          left_q_.pop_front();
          right_q_.pop_front();
        } else if (tl < tr) {
          // 左目过旧：丢掉它，保留右目等下一帧左目
          left_q_.pop_front();
          ++n_skew_;
        } else {
          right_q_.pop_front();
          ++n_skew_;
        }
      }
    }
    if (pairs.empty()) {
      return;
    }
    // [NP01-FIX15] 闸门关闭时：配对结果直接丢弃，绝不喂给 SLAM，也不消费 IMU。
    // 这样 SLAM 永远看不到地面积压帧，第一帧就是闸门打开后的新鲜帧。
    if (!initGateAllowsFrames(
          pairs.size(), stampSeconds(pairs.front().first->header.stamp)))
    {
      n_gate_skip_ += static_cast<int>(pairs.size());
      return;
    }
    for (const auto & p : pairs) {
      processPair(p.first, p.second);
    }
  }

  void processPair(
    const sensor_msgs::msg::Image::ConstSharedPtr & l_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & r_msg)
  {
    ++n_pairs_;

    if (l_msg->width != r_msg->width || l_msg->height != r_msg->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "左右目尺寸不一致 %ux%u vs %ux%u",
        l_msg->width, l_msg->height, r_msg->width, r_msg->height);
      return;
    }

    double tl = stampSeconds(l_msg->header.stamp);
    // [SIM-ONLY 同源改造] D435i 是硬件同步双目，左右目同戳。
    // Gazebo 两个相机分别渲染，header.stamp 有微小差；在驱动层统一为左目时间戳，
    // 下游就不需要 5 ms 配对容差，也不会因 skew 丢帧。
    const double tr = tl;
    if (tl <= 0.0) {
      ++n_zero_stamp_;
      return;
    }

    // 左右目/IMU 先过同一个 SensorClock；任何一路时钟域跳变都重置 epoch。
    bool ready = false;
    uint64_t epoch = 0;
    int64_t epoch_us = 0;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      const bool left_ok = acceptLocked(0, tl);
      const bool right_ok = acceptLocked(1, tr);
      ready = left_ok && right_ok && sensor_clock_.Ready();
      epoch = sensor_clock_.Epoch();
      if (ready) {
        epoch_us = simToEpochUsLocked(static_cast<int64_t>(std::llround(tl * 1e6)));
        // [PHASE5-OBS] cache the exact mapping used for THIS frame, under the
        // same clock_mutex_ lock as epoch_us. Observation only.
        if (obs_enabled_) {  // [PHASE7-DIAG]
          obs_sample_us_ = epoch_us;
          obs_offset_us_ = offset_us_;
          obs_offset_valid_ = have_offset_;
        }
      }
    }
    if (obs_enabled_) {  // [PHASE7-DIAG] observation only
      obs_clock_epoch_ = epoch;
      obs_jumps_ = time_jumps_;
    }
    if (trace_) trace_->Mark(g_pending_trace_seq, ready ? 1 : 2,
                            ready ? 0 : 2,
                            ORB_SLAM3::px4::SystemMicros(),
                            ORB_SLAM3::px4::SteadyMicros(),
                            offset_us_, epoch);
    if (!ready) {
      ++n_clock_reject_;
      return;
    }

    bool reset_map = false;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      if (clock_jump_pending_) {
        clock_jump_pending_ = false;
        reset_map = true;
      }
      if (epoch != processed_epoch_) {
        processed_epoch_ = epoch;
        reset_map = true;
      }
    }
    if (reset_map) {
      RCLCPP_WARN(get_logger(), "sensor clock epoch -> %llu; resetting active map",
        static_cast<unsigned long long>(epoch));
      slam_->ResetActiveMap();
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_buf_.clear();
      return;
    }

    if (tl <= last_slam_t_) {
      ++n_time_regress_;
      return;
    }

    // [FIX3] 图像转换/灰度化必须在消费 imu_buf_ 之前完成。
    // FIX2 正好相反：先 pop 样本、后 imageToMat()，转换失败时样本
    // 既没交给算法也没留在缓存（跳过图像却清空了积分所需样本）。
    cv::Mat im_left;
    cv::Mat im_right;
    try {
      im_left = imageToMat(*l_msg);
      im_right = imageToMat(*r_msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "图像转换失败: %s", e.what());
      return;
    }

    // 统一转灰度后再交给 TrackStereo：InputRecording 只接受 CV_8UC1，
    // 而 Tracking::GrabImageStereo 在 Camera.RGB:1 下本来就会做等价的
    // COLOR_RGB2GRAY，所以算法输入不变。
    if (im_left.channels() == 3) {
      cv::cvtColor(im_left, im_left, cv::COLOR_RGB2GRAY);
      cv::cvtColor(im_right, im_right, cv::COLOR_RGB2GRAY);
    }

    // [FIX3] 覆盖屏障：按「本帧区间是否被 IMU 时间覆盖」判定，而不是样本数量。
    // ORB_SLAM3 Tracking::PreintegrateIMU() 的契约（Tracking.cc:1557-1596）：
    //   消费  [prev_ts - mImuPer, cur_ts - mImuPer) 的样本 → mvImuFromLastFrame
    //   保留  第一个 >= cur_ts - mImuPer 的样本（末端边界，留给下一帧）
    //   n = size() - 1；n == 0 则 Empty IMU measurements vector!!! 并返回
    // 因此可积分的充要条件是：既有可消费样本、又有末端边界样本。
    // 旧 FIX2 用 available<2（数量）当屏障，两个同数量样本可以全是帧首样本，
    // 末端无覆盖，照样失败（见 evidence/integral_coverage_audit.md §2）。
    // 本屏障只等待/判定时间覆盖：不合成、不复制、不插值、不推进任何时戳。
    if (require_imu_ && !waitImuCoverage(last_slam_t_, tl)) {
      ++n_no_imu_;
      if (n_no_imu_ == 1 || n_no_imu_ % 120 == 0) {
        RCLCPP_WARN(get_logger(),
          "图像对 %.3f 的 IMU 区间覆盖未成立（front=%d back=%d 可消费=%zu）；"
          "跳过该帧并保留全部样本（累计 %d）",
          tl, static_cast<int>(cov_front_), static_cast<int>(cov_back_), cov_n_,
          n_no_imu_);
      }
      return;
    }

    std::vector<ORB_SLAM3::IMU::Point> v_imu;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      while (!imu_buf_.empty() && imu_buf_.front().t <= tl + 1e-6) {
        const auto & s = imu_buf_.front();
        v_imu.emplace_back(s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.t);
        imu_buf_.pop_front();
      }
    }

    if (n_pairs_ <= 3) {
      cv::Scalar mL, sL, mR, sR;
      cv::meanStdDev(im_left, mL, sL);
      cv::meanStdDev(im_right, mR, sR);
      RCLCPP_INFO(get_logger(),
        "[probe] pair#%d %dx%d  L mean=%.1f std=%.1f | R mean=%.1f std=%.1f | imu=%zu",
        n_pairs_, im_left.cols, im_left.rows, mL[0], sL[0], mR[0], sR[0], v_imu.size());
    }

    const uint64_t rx_mono_us = ORB_SLAM3::px4::SteadyMicros();
    const uint64_t rx_wall_us = ORB_SLAM3::px4::SystemMicros();
    const uint64_t track_start_us = rx_mono_us;
    const Sophus::SE3f Tcw = slam_->TrackStereo(im_left, im_right, tl, v_imu);
    last_slam_t_ = tl;

    const int state = slam_->GetTrackingState();
    if (obs_enabled_ && latency_.is_open()) {
      // [PHASE5-OBS] Observation only: does not touch sample timestamps,
      // the Bridge queue, Gate decisions or acceptance conditions.
      // No zero-filling of unknown values; no clamping of negative ages.
      const uint64_t obs_track_end = ORB_SLAM3::px4::SteadyMicros();
      const uint64_t obs_submit_wall = ORB_SLAM3::px4::SystemMicros();
      const uint64_t obs_submit_mono = ORB_SLAM3::px4::SteadyMicros();
      const auto obs_nav = slam_->GetNavigationState();  // const, read-only
      const long long obs_src = static_cast<long long>(std::llround(tl * 1e6));
      latency_ << n_pairs_ << "," << obs_src
               << "," << rx_mono_us << "," << rx_wall_us << "," << track_start_us
               << "," << obs_track_end << "," << v_imu.size()
               << "," << state << ","
               << "accepted" << ",ok"
               << "," << obs_src
               << ",";
      if (obs_sample_us_ > 0) latency_ << obs_sample_us_;  // else EMPTY, not 0
      latency_ << ",";
      if (obs_offset_valid_) latency_ << obs_offset_us_;   // else EMPTY
      latency_ << "," << (obs_offset_valid_ ? 1 : 0)
               << "," << obs_clock_epoch_ << "," << obs_nav.referenceEpoch
               << "," << n_pairs_ << "," << obs_submit_wall << ","
               << obs_submit_mono << ","
               << (obs_sample_us_ > 0 ? 1 : 0) << "," << obs_jumps_ << "\n";
    }
    std_msgs::msg::Int32 s_msg;
    s_msg.data = state;
    state_pub_->publish(s_msg);

    if (state != static_cast<int>(ORB_SLAM3::Tracking::OK) &&
      state != static_cast<int>(ORB_SLAM3::Tracking::OK_KLT))
    {
      ++n_lost_;
      if (n_lost_ == 1 || n_lost_ % 90 == 0) {
        RCLCPP_WARN(get_logger(), "未跟踪成功 state=%s (累计 %d 帧)",
          stateName(state), n_lost_);
      }
      // 仍然把无效样本交给 Px4Bridge：Gate 需要看到 tracking_invalid 才会
      // 及时撤销，而不是因为“没有新消息”被动超时。
      auto nav = slam_->GetNavigationState();
      bridge_->Submit(nav, static_cast<uint64_t>(epoch_us), epoch,
                      makeSubmitMeta(nav, Tcw, v_imu, tl));
      return;
    }

    ++n_ok_;
    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f p = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = l_msg->header.stamp;
    out.header.frame_id = "slam_world";
    out.pose.position.x = p.x();
    out.pose.position.y = p.y();
    out.pose.position.z = p.z();
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
    pose_pub_->publish(out);

    auto nav = slam_->GetNavigationState();
    // [SIM-ONLY，默认关闭] 驱动层速度预检。它改变的是「交给 Gate 的样本」，
    // 与实机路径不同；只有显式设置 sim_nav_guard:=true 才启用。
    if (sim_nav_guard_) {
      constexpr double kMaxAccel = 30.0;
      const double nt = nav.timestamp;
      if (nav.valid && nav.velocityValid && nt > 0.0) {
        const Eigen::Vector3f v = nav.velocityWorld;
        bool bad = !v.allFinite();
        double accel = 0.0;
        if (!bad && nav_guard_has_good_) {
          const double dt = nt - nav_guard_last_t_;
          if (dt > 0.0 && dt < 0.5) {
            accel = (v - nav_guard_last_v_).norm() / dt;
            if (!std::isfinite(accel) || accel > kMaxAccel) bad = true;
          }
        }
        if (bad) {
          ++nav_guard_rejects_;
          nav_guard_cooldown_ = 2;
          RCLCPP_WARN(get_logger(),
            "[navguard] t=%.4f accel=%.1f finite=%d -> invalid n=%d",
            nt, accel, static_cast<int>(v.allFinite()), nav_guard_rejects_);
        } else {
          nav_guard_has_good_ = true;
          nav_guard_last_t_ = nt;
          nav_guard_last_v_ = v;
        }
      }
      if (nav_guard_cooldown_ > 0) {
        --nav_guard_cooldown_;
        nav.valid = false;
        nav_guard_has_good_ = false;
      }
    }
    bridge_->Submit(nav, static_cast<uint64_t>(epoch_us), epoch,
                    makeSubmitMeta(nav, Tcw, v_imu, tl));

    if (n_ok_ % 30 == 0) {
      RCLCPP_INFO(get_logger(),
        "tracking OK #%d t=(%.2f, %.2f, %.2f) imu=%zu nav(valid=%d init=%d ba2=%d vel=%d)",
        n_ok_, p.x(), p.y(), p.z(), v_imu.size(), static_cast<int>(nav.valid),
        static_cast<int>(nav.inertialInitialized), static_cast<int>(nav.inertialBA2),
        static_cast<int>(nav.velocityValid));
    }
  }

  // [NP-01 FIX11][OBSERVATION-ONLY] One value-copy snapshot of everything the
  // NavigationPose / raw SLAM / IMU owner knows for THIS frame. Built from the
  // same GetNavigationState() copy and the same v_imu bundle that TrackStereo
  // consumed. Not used by SensorClock, Gate, Bridge decisions or sample times;
  // truth is never read here.
  ORB_SLAM3::px4::SubmitMeta makeSubmitMeta(
      const ORB_SLAM3::System::NavigationState& nav,
      const Sophus::SE3f& Tcw,
      const std::vector<ORB_SLAM3::IMU::Point>& imu,
      double srcTimeSeconds) const
  {
    ORB_SLAM3::px4::SubmitMeta m;
    m.pairId = static_cast<uint64_t>(n_pairs_);
    m.srcSimUs = static_cast<int64_t>(std::llround(srcTimeSeconds * 1e6));
    m.mapOffsetUs = obs_offset_valid_ ? obs_offset_us_ : 0;
    m.mapId = nav.mapId;
    m.initialKeyFrameId = nav.initialKeyFrameId;
    m.frameId = nav.frameId;
    m.mapToWorld = nav.mapToWorld.rotationMatrix().cast<double>();
    m.mapTranslation = nav.mapToWorld.translation().cast<double>();
    m.initialImuToWorld = nav.initialImuToWorld.rotationMatrix().cast<double>();
    m.initialImuTranslation = nav.initialImuToWorld.translation().cast<double>();
    const Sophus::SE3f TwcRaw = Tcw.inverse();   // raw SLAM pose, MAP frame
    m.rawCameraToWorld = TwcRaw.rotationMatrix().cast<double>();
    m.rawCameraPosition = TwcRaw.translation().cast<double>();
    m.imuCount = static_cast<uint64_t>(imu.size());
    if(imu.size() >= 2) {
      m.gyroSpanS = imu.back().t - imu.front().t;
      Eigen::Vector3d acc = Eigen::Vector3d::Zero();
      bool any = false;
      for(size_t i = 1; i < imu.size(); ++i) {
        const double dt = imu[i].t - imu[i-1].t;
        if(!(dt > 0.0) || dt > 0.1) continue;
        acc += 0.5 * (imu[i].w.cast<double>() + imu[i-1].w.cast<double>()) * dt;
        any = true;
      }
      if(any) m.gyroIntegral = acc;
    }
    return m;
  }

  void report()
  {
    const int st = slam_ ? slam_->GetTrackingState() : -1;
    std_msgs::msg::Int32 s_msg;
    s_msg.data = st;
    state_pub_->publish(s_msg);

    const size_t pub_l = count_publishers(left_topic_);
    const size_t pub_r = count_publishers(right_topic_);
    const size_t pub_i = count_publishers(imu_topic_);

    double offset_s = 0.0;
    uint64_t epoch = 0;
    bool clock_ready = false;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      offset_s = offset_us_ * 1e-6;
      epoch = sensor_clock_.Epoch();
      clock_ready = sensor_clock_.Ready();
    }
    bool gate_open = false;
    std::string gate_reason;
    int gate_skip = 0;
    int land_landed = -1, land_maybe = -1, land_contact = -1;
    double land_age_s = -1.0;
    {
      std::lock_guard<std::mutex> glock(gate_mutex_);
      gate_open = gate_open_;
      gate_reason = gate_block_reason_;
      gate_skip = n_gate_skip_;
      land_landed = static_cast<int>(land_landed_);
      land_maybe = static_cast<int>(land_maybe_landed_);
      land_contact = static_cast<int>(land_ground_contact_);
      land_age_s = land_last_mono_us_
        ? static_cast<double>(ORB_SLAM3::px4::SteadyMicros() - land_last_mono_us_) * 1e-6
        : -1.0;
    }
    RCLCPP_INFO(get_logger(),
      "[stat] img L=%d R=%d imu=%d | pubs %zu/%zu/%zu | pairs=%d ok=%d lost=%d skew=%d "
      "no_imu=%d zero_stamp=%d time_regress=%d clock_reject=%d imu_reorder=%d | "
      "reject_why{domainFault=%d stalled=%d srcFault=%d stale=%d host=%d other=%d} | "
      "clock ready=%d epoch=%llu "
      "offset=%+.3fs jumps=%d | gate=%s(%s) skipped=%d "
      "land(landed=%d maybe=%d contact=%d age=%.2fs) | state=%s",
      n_left_, n_right_, n_imu_, pub_l, pub_r, pub_i, n_pairs_, n_ok_, n_lost_, n_skew_,
      n_no_imu_, n_zero_stamp_, n_time_regress_, n_clock_reject_, n_imu_reorder_,
      n_clock_reject_reason_[0], n_clock_reject_reason_[1], n_clock_reject_reason_[2],
      n_clock_reject_reason_[3], n_clock_reject_reason_[4], n_clock_reject_reason_[5],
      static_cast<int>(clock_ready),
      static_cast<unsigned long long>(epoch), offset_s, time_jumps_,
      gate_open ? "OPEN" : "CLOSED",
      gate_reason.empty() ? "-" : gate_reason.c_str(), gate_skip,
      land_landed, land_maybe, land_contact, land_age_s,
      stateName(st));

    if (pub_l == 0 || pub_r == 0 || pub_i == 0) {
      RCLCPP_WARN(get_logger(),
        "看不到图像/IMU 发布者（pubs %zu/%zu/%zu）：确认 ros_gz_bridge 与节点在同一 "
        "ROS_DOMAIN_ID", pub_l, pub_r, pub_i);
    } else if (n_pairs_ > 0 && n_ok_ == 0) {
      RCLCPP_WARN(get_logger(),
        "有图像但从未跟踪成功（state=%s）：检查纹理、左右目同步、IMU 时间戳",
        stateName(st));
    }
  }

  void perfReport()
  {
    const uint64_t now = ORB_SLAM3::px4::SteadyMicros();
    if (last_perf_us_ == 0) {
      last_perf_us_ = now;
      last_perf_pairs_ = n_pairs_;
      last_perf_ok_ = n_ok_;
      last_perf_imu_ = n_imu_;
      return;
    }
    const double dt = (now - last_perf_us_) * 1e-6;
    if (dt <= 0.0) {
      return;
    }
    const auto nav = slam_->GetNavigationState();
    RCLCPP_INFO(get_logger(),
      "[VIO_PERF] pairs_hz=%.1f ok_hz=%.1f imu_hz=%.1f features=%zu tracking=%d "
      "map=%lu ref_epoch=%llu clock_epoch=%llu nav(valid=%d init=%d ba2=%d vel=%d)",
      (n_pairs_ - last_perf_pairs_) / dt, (n_ok_ - last_perf_ok_) / dt,
      (n_imu_ - last_perf_imu_) / dt, slam_->GetTrackedKeyPointsUn().size(),
      slam_->GetTrackingState(), static_cast<unsigned long>(nav.mapId),
      static_cast<unsigned long long>(nav.referenceEpoch),
      static_cast<unsigned long long>(processed_epoch_),
      static_cast<int>(nav.valid), static_cast<int>(nav.inertialInitialized),
      static_cast<int>(nav.inertialBA2), static_cast<int>(nav.velocityValid));
    last_perf_us_ = now;
    last_perf_pairs_ = n_pairs_;
    last_perf_ok_ = n_ok_;
    last_perf_imu_ = n_imu_;
  }

  std::string vocab_file_, settings_file_, px4_config_;
  std::string left_topic_, right_topic_, imu_topic_, clock_topic_;
  std::string pose_topic_, state_topic_;
  std::string record_dir_, latency_csv_, trace_csv_;
  std::unique_ptr<TimeTrace> trace_;
  bool use_viewer_{false};
  bool sim_nav_guard_{false};
  double sync_tol_{0.005};
  int max_imu_buffer_{5000};
  double offset_window_s_{2.0};
  bool require_imu_{true};

  bool nav_guard_has_good_{false};
  double nav_guard_last_t_{0.0};
  Eigen::Vector3f nav_guard_last_v_{Eigen::Vector3f::Zero()};
  int nav_guard_rejects_{0};
  int nav_guard_cooldown_{0};

  std::unique_ptr<ORB_SLAM3::System> slam_;
  std::unique_ptr<ORB_SLAM3::px4::Bridge> bridge_;
  std::shared_ptr<ORB_SLAM3::InputRecording> recording_;
  std::ofstream latency_;
  ORB_SLAM3::px4::SensorClock sensor_clock_;

  rclcpp::CallbackGroup::SharedPtr image_group_, imu_group_, clock_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_, right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;

  // ---- [NP01-FIX15] 初始化闸门状态 ----
  std::string init_gate_{"off"};
  std::string land_topic_;
  double init_gate_airborne_hold_{0.50};
  double init_gate_health_window_{1.00};
  double init_gate_min_img_hz_{15.0};
  double init_gate_max_first_age_ms_{250.0};
  std::mutex gate_mutex_;
  bool gate_open_{false};
  std::string gate_block_reason_{"landed"};
  uint64_t gate_opened_us_{0};
  uint64_t gate_open_epoch_{0};
  bool land_landed_{true}, land_maybe_landed_{true}, land_ground_contact_{true};
  uint64_t land_last_mono_us_{0};
  uint64_t airborne_since_us_{0};
  std::deque<uint64_t> left_rx_us_, right_rx_us_, imu_rx_us_;
  int n_gate_skip_{0}, n_gate_skip_img_l_{0}, n_gate_skip_img_r_{0};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_, perf_timer_;

  std::mutex image_mutex_, imu_mutex_, clock_mutex_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> left_q_, right_q_;
  std::deque<ImuSample> imu_buf_;
  std::deque<std::pair<uint64_t, int64_t>> offset_history_;

  bool have_offset_{false};
  int64_t offset_us_{0};
  // [PHASE7-DIAG] Runtime observation switch (default ON = phase5 behaviour).
  bool obs_enabled_{true};
  // [PHASE5-OBS] Mapping parameters actually used for the current frame,
  // cached under clock_mutex_ for the observation columns. Never read by
  // SensorClock / Gate / Bridge and never written back into sample times.
  int64_t obs_sample_us_{0};
  int64_t obs_offset_us_{0};
  bool obs_offset_valid_{false};
  uint64_t obs_clock_epoch_{0};
  int obs_jumps_{0};
  std::deque<std::pair<uint64_t, int64_t>> offset_window_;
  int time_jumps_{0};
  bool clock_jump_pending_{false};
  uint64_t processed_epoch_{0};
  int64_t last_clock_sim_us_{0};
  uint64_t last_clock_rx_us_{0};

  double last_slam_t_{-1.0};
  double last_imu_t_{-1.0};
  double last_stream_ts_[4] = {-1.0, -1.0, -1.0, -1.0};
  uint64_t last_stream_mono_[4] = {0, 0, 0, 0};
  uint64_t last_perf_us_{0};
  uint64_t last_perf_pairs_{0}, last_perf_ok_{0}, last_perf_imu_{0};

  // [FIX3] 最近一次覆盖判定的结果（只用于日志/取证，不参与判定）。
  bool cov_front_{false}, cov_back_{false};
  size_t cov_n_{0};

  int n_left_{0}, n_right_{0}, n_imu_{0}, n_pairs_{0}, n_ok_{0}, n_lost_{0}, n_skew_{0};
  int n_no_imu_{0}, n_zero_stamp_{0}, n_time_regress_{0}, n_clock_reject_{0};
  int n_imu_dropped_{0}, n_imu_reorder_{0};
  // [NP01-FIX15] 拒绝原因聚合桶，顺序见 reasonIndex()。
  int n_clock_reject_reason_[6] = {0, 0, 0, 0, 0, 0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<StereoInertialSim>();
    // [SIM-ONLY] 第 24 轮单变量：执行器线程 4 -> 8。
    // 实测每轮必有一次 gyro 回调停更 220~261ms（dt 恒 4ms），
    // 且 Tracking.cc:1783 的 1s 时间戳跳变反复触发；4 线程下图像回调
    // 长任务可能占满线程池。只改并发度，不动 callback group 与消息顺序。
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 8);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    std::fprintf(stderr, "stereo_inertial_sim_node: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
