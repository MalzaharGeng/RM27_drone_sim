// RM27 / [DIAG-ONLY] 跟踪短时失锁诊断（默认关闭）
//
// 目的：在不改变算法、不改动 Gate 判定、不改变提交时序的前提下，记录每一帧
// 「跟踪线程实际看到的东西」与「实际提交给 Px4Bridge 的导航快照」之间的差异，
// 用来区分下列互斥假设：
//
//   kind=0  state==OK 且 nav.valid      —— 正常帧（应被 Gate 接受）
//   kind=1  state!=OK                   —— ORB-SLAM3 真实非跟踪态（含 RECENTLY_LOST）
//   kind=2  state==OK 但 nav.valid==false —— 同一帧两种消费者判断不一致（软件语义分叉）
//   kind=3  state==OK、nav.valid 曾为真，但被 sim_nav_guard 置为 false
//
// 开销约束（对应 tracking_diagnostic_contract.md）：
//   * 跟踪线程只做「拷一行结构体进有界环形缓冲」，不写文件；短时持有队列互斥锁；
//   * 后台线程每 250 ms 批量落盘一次，批量写，不逐帧 flush；
//   * 环形缓冲有硬上限，溢出时丢最旧并计数（overflows），丢弃数写入退出汇总行；
//   * 未设置环境变量时 init() 不做任何事，frame() 立即返回（直接返回）。
//
// 本文件不参与控制、不参与健康判定、不进入任何验收判据。
#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace rm27 {

struct TrackDiagRow {
    // —— 身份与时间 ——
    uint64_t seq = 0;             // 节点内提交序号（1 递增，每个 TrackStereo 结果一行）
    uint64_t frame_id = 0;        // ORB-SLAM3 当前帧 mnId（nav.frameId）
    int64_t  src_sim_us = 0;      // 图像时间戳（仿真时钟，tl*1e6）
    int64_t  rx_mono_us = 0;      // 收到图像对的单调时钟
    int64_t  submit_epoch_us = 0; // 提交给 Bridge 的 epoch 时间戳
    double   track_ms = -1.0;     // TrackStereo 耗时
    // —— 跟踪侧 ——
    int      tracking_state = -1; // ORB_SLAM3 Tracking::eTrackingState
    int      tracked_kp = -1;     // GetTrackedKeyPointsUn().size()：当前帧去畸变 ORB 关键点容器大小
    int      n_imu = -1;          // 本帧消费的 IMU 样本数
    int      imu_buf_depth = -1;  // 消费后 imu_buf_ 剩余深度
    // —— 导航快照侧（实际提交给 Gate 的那份） ——
    int      nav_valid = -1;
    int      nav_tracking_state = -1;
    int      nav_imu_init = -1;
    int      nav_ba2 = -1;
    int      nav_vel_valid = -1;
    uint64_t map_id = 0;
    uint64_t ref_epoch = 0;
    uint64_t clock_epoch = 0;
    // —— 分类与累计 ——
    int      kind = -1;
    int      navguard_fired = 0;
    int      n_ok = 0;
    int      n_lost = 0;
    int      n_pairs = 0;
};

class TrackDiag {
public:
    static TrackDiag& instance();

    // 仅当 dir 非空且尚未初始化时生效；失败只打印，不抛出。
    void init(const std::string& dir);
    bool enabled() const { return enabled_.load(std::memory_order_relaxed); }

    // 跟踪线程调用：只入队，不起线程、不碰文件。
    void frame(const TrackDiagRow& r);

    // 退出时冲刷并关文件；可重复调用。
    void shutdown();

    uint64_t rows() const { return rows_.load(std::memory_order_relaxed); }
    uint64_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

    TrackDiag(const TrackDiag&) = delete;
    TrackDiag& operator=(const TrackDiag&) = delete;

private:
    TrackDiag() = default;
    ~TrackDiag();
    void writerLoop();
    void flushLocked(std::vector<TrackDiagRow>& out);

    std::atomic<bool> enabled_{false};
    std::atomic<bool> stop_{false};
    std::atomic<uint64_t> rows_{0};
    std::atomic<uint64_t> dropped_{0};

    std::mutex mu_;
    std::deque<TrackDiagRow> buf_;
    std::thread writer_;
    std::FILE* fp_{nullptr};

    static constexpr size_t kMaxBuffered = 20000;
};
}  // namespace rm27
