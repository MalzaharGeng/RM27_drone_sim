// RM27 仿真：双目 ORB_SLAM3 节点
//
// 订阅 gz 桥出的左右目图像 → ORB_SLAM3::System::TrackStereo → 发布位姿
// 下游由 slam_pose_to_px4.py 转 NED 并注入 PX4。
//
// 设计要点：
//  * 图像必须深拷贝，ORB_SLAM3 会持有引用；
//  * 时间戳只在有效时才用于去重（gz 若给 0 时间戳，退化为节点时钟），
//    否则同一帧会被反复丢弃、表现成"完全没有输出"；
//  * 每 3 秒打印一次计数器，用来区分「没收到图」和「收到图但没初始化」——
//    这两种情况以前都是静默的，无法排查。
#include <memory>
#include <string>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include "System.h"

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
  cv::Mat out = wrapped.clone();   // ORB_SLAM3 会持有，必须深拷贝
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
}  // namespace

class StereoSlamNode : public rclcpp::Node
{
public:
  StereoSlamNode()
  : Node("stereo_slam_node")
  {
    const char * home_env = std::getenv("HOME");
    const auto home = std::string(home_env ? home_env : "");
    vocab_file_ = declare_parameter<std::string>("vocab_file", home + "/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    settings_file_ = declare_parameter<std::string>(
      "settings_file", home + "/ORB_SLAM3/config/RM27_Sim_D435i_Stereo.yaml");
    left_topic_ = declare_parameter<std::string>(
      "left_topic", "/world/rm27_world/model/x500_mono_cam_0/link/cam_left_link/sensor/cam_left/image");
    right_topic_ = declare_parameter<std::string>(
      "right_topic", "/world/rm27_world/model/x500_mono_cam_0/link/cam_right_link/sensor/cam_right/image");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/slam/pose");
    use_viewer_ = declare_parameter<bool>("use_viewer", false);
    sync_tol_ = declare_parameter<double>("sync_tolerance", 0.05);
    // 非空时把最初几对图像存成 PNG，用于确认相机到底拍到了什么
    dump_dir_ = declare_parameter<std::string>("dump_dir", "");

    RCLCPP_INFO(get_logger(), "vocab: %s", vocab_file_.c_str());
    RCLCPP_INFO(get_logger(), "settings: %s", settings_file_.c_str());

    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocab_file_, settings_file_, ORB_SLAM3::System::STEREO, use_viewer_);

    // 仿真里 ros_gz_bridge 的 Image 发布端是 reliable。若订阅用
    // SensorDataQoS（best_effort），1360x960 大帧在 Fast DDS 上会大量丢包
    // （实测只剩 ~3 Hz）；改成 reliable 后可稳定收到 ~28 Hz。
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
    left_sub_ = create_subscription<sensor_msgs::msg::Image>(
      left_topic_, img_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onLeft(m);});
    right_sub_ = create_subscription<sensor_msgs::msg::Image>(
      right_topic_, img_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onRight(m);});

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    state_pub_ = create_publisher<std_msgs::msg::Int32>("/slam/tracking_state", 10);

    status_timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {report();});

    RCLCPP_INFO(get_logger(), "left : %s", left_topic_.c_str());
    RCLCPP_INFO(get_logger(), "right: %s", right_topic_.c_str());
    RCLCPP_INFO(get_logger(), "pose : %s", pose_topic_.c_str());
  }

  ~StereoSlamNode() override
  {
    if (slam_) {
      slam_->Shutdown();
      slam_->SaveTrajectoryTUM("/tmp/slam_trajectory_tum.txt");
      RCLCPP_INFO(get_logger(), "trajectory saved: /tmp/slam_trajectory_tum.txt");
    }
  }

private:
  double stamp(const builtin_interfaces::msg::Time & t) const
  {
    return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
  }

  void report()
  {
    const int st = slam_ ? slam_->GetTrackingState() : -1;
    std_msgs::msg::Int32 s_msg;
    s_msg.data = st;
    state_pub_->publish(s_msg);

    // 发布者计数是判断"DDS 有没有匹配上"的直接依据：
    // 0 说明 ROS 图里看不到桥的 publisher（发现没建立）；
    // >0 但收不到消息则是 QoS 或 gz 侧无数据。
    const size_t pub_l = count_publishers(left_topic_);
    const size_t pub_r = count_publishers(right_topic_);

    RCLCPP_INFO(get_logger(),
      "[stat] img L=%d R=%d | pubs %zu/%zu | pairs=%d ok=%d lost=%d skew=%d | state=%s",
      n_left_, n_right_, pub_l, pub_r, n_pairs_, n_ok_, n_lost_, n_skew_, stateName(st));

    if (pub_l == 0 || pub_r == 0) {
      RCLCPP_WARN(get_logger(),
        "看不到图像发布者（pubs %zu/%zu）：DDS 发现没建立。确认桥和节点在同一 "
        "ROS_DOMAIN_ID，且都设置了 ROS_LOCALHOST_ONLY=1", pub_l, pub_r);
    } else if (n_left_ == 0 || n_right_ == 0) {
      RCLCPP_WARN(get_logger(),
        "发布者可见（pubs %zu/%zu）但收不到图像：检查 QoS 兼容性与 gz 侧是否真在出图",
        pub_l, pub_r);
    } else if (n_pairs_ > 0 && n_ok_ == 0) {
      RCLCPP_WARN(get_logger(),
        "有图像但从未跟踪成功（state=%s）：多为画面无纹理/全黑，或左右目未同步",
        stateName(st));
    }
  }

  void onLeft(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    ++n_left_;
    if (n_left_ == 1) {
      RCLCPP_INFO(get_logger(), "first LEFT  %ux%u enc=%s stamp=%.3f",
        msg->width, msg->height, msg->encoding.c_str(), stamp(msg->header.stamp));
    }
    left_ = msg;
    tryTrack();
  }

  void onRight(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    ++n_right_;
    if (n_right_ == 1) {
      RCLCPP_INFO(get_logger(), "first RIGHT %ux%u enc=%s stamp=%.3f",
        msg->width, msg->height, msg->encoding.c_str(), stamp(msg->header.stamp));
    }
    right_ = msg;
    tryTrack();
  }

  void tryTrack()
  {
    if (!left_ || !right_) {return;}

    auto l_msg = left_;
    auto r_msg = right_;
    left_.reset();
    right_.reset();
    ++n_pairs_;

    if (l_msg->width != r_msg->width || l_msg->height != r_msg->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "左右目尺寸不一致 %ux%u vs %ux%u",
        l_msg->width, l_msg->height, r_msg->width, r_msg->height);
      return;
    }

    const double tl = stamp(l_msg->header.stamp);
    const double tr = stamp(r_msg->header.stamp);
    if (tl > 1e-6 && tr > 1e-6 && std::abs(tl - tr) > sync_tol_) {
      ++n_skew_;
      if (n_skew_ == 1 || n_skew_ % 120 == 0) {
        RCLCPP_WARN(get_logger(),
          "左右目时间戳差 %.3fs 超过容差 %.3fs（累计 %d 次）",
          std::abs(tl - tr), sync_tol_, n_skew_);
      }
      return;
    }

    // 时间戳有效才用于去重；无效（0）时退化为节点时钟，且保证严格递增，
    // 避免 ORB_SLAM3 因时间戳不前进而拒绝建关键帧。
    double t = (tl > 1e-6) ? tl : tr;
    if (t <= 1e-6) {t = now().seconds();}
    if (t <= last_stamp_) {t = last_stamp_ + 1.0 / 30.0;}
    last_stamp_ = t;

    cv::Mat im_left, im_right;
    try {
      im_left = imageToMat(*l_msg);
      im_right = imageToMat(*r_msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "图像转换失败: %s", e.what());
      return;
    }

    // 前 3 对图像做一次亮度统计：std 接近 0 说明画面是纯色/全黑，
    // 这时 ORB 提不出特征、永远停在 NOT_INITIALIZED，光看计数器无法区分。
    if (n_pairs_ <= 3) {
      cv::Scalar mL, sL, mR, sR;
      cv::meanStdDev(im_left, mL, sL);
      cv::meanStdDev(im_right, mR, sR);
      RCLCPP_INFO(get_logger(),
        "[probe] pair#%d %dx%d  L mean=%.1f std=%.1f | R mean=%.1f std=%.1f"
        "  (std<2 => 画面纯色/全黑)",
        n_pairs_, im_left.cols, im_left.rows, mL[0], sL[0], mR[0], sR[0]);

      if (!dump_dir_.empty()) {
        char p1[512], p2[512];
        std::snprintf(p1, sizeof(p1), "%s/left_%03d.png", dump_dir_.c_str(), n_pairs_);
        std::snprintf(p2, sizeof(p2), "%s/right_%03d.png", dump_dir_.c_str(), n_pairs_);
        if (cv::imwrite(p1, im_left) && cv::imwrite(p2, im_right)) {
          RCLCPP_INFO(get_logger(), "[probe] 已存图 %s / %s", p1, p2);
        } else {
          RCLCPP_WARN(get_logger(), "[probe] 存图失败，检查 dump_dir=%s 是否存在",
            dump_dir_.c_str());
        }
      }
    }

    Sophus::SE3f Tcw = slam_->TrackStereo(im_left, im_right, t);
    const int state = static_cast<int>(slam_->GetTrackingState());

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
      return;
    }

    ++n_ok_;
    const Sophus::SE3f Twc = Tcw.inverse();   // Tcw 是世界→相机
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

    if (n_ok_ % 30 == 0) {
      RCLCPP_INFO(get_logger(), "tracking OK #%d  t=(%.2f, %.2f, %.2f)",
        n_ok_, p.x(), p.y(), p.z());
    }
  }

  std::string vocab_file_, settings_file_, left_topic_, right_topic_, pose_topic_, dump_dir_;
  bool use_viewer_{false};
  double sync_tol_{0.05};
  double last_stamp_{-1.0};

  int n_left_{0}, n_right_{0}, n_pairs_{0}, n_ok_{0}, n_lost_{0}, n_skew_{0};

  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_, right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  sensor_msgs::msg::Image::ConstSharedPtr left_, right_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StereoSlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
