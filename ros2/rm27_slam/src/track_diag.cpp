#include "track_diag.hpp"

#include <chrono>
#include <cstring>
#include <iostream>

namespace rm27 {

TrackDiag& TrackDiag::instance()
{
    static TrackDiag inst;
    return inst;
}

TrackDiag::~TrackDiag()
{
    shutdown();
}

void TrackDiag::init(const std::string& dir)
{
    if (dir.empty()) {
        return;
    }
    if (enabled_.load()) {
        return;
    }
    const std::string path = dir + "/track_frames.csv";
    fp_ = std::fopen(path.c_str(), "w");
    if (!fp_) {
        std::cerr << "[TRACKDIAG] cannot open " << path << ": " << std::strerror(errno) << std::endl;
        return;
    }
    // 文件头：字段语义 + 缓冲上限，便于离线核对（不写版本化 schema，只写自描述）
    std::fprintf(fp_,
        "# rm27 track diag v1  [DIAG-ONLY][SAME] 不参与控制/健康/验收\n"
        "# kind: 0=ok  1=state!=OK  2=state==OK but nav.valid=false  3=navguard\n"
        "# tracked_kp = GetTrackedKeyPointsUn().size() (当前帧去畸变 ORB 关键点容器大小，非匹配内点数)\n"
        "# 环形缓冲上限=%zu，溢出丢最旧\n", kMaxBuffered);
    std::fprintf(fp_,
        "seq,frame_id,src_sim_us,rx_mono_us,submit_epoch_us,track_ms,tracking_state,tracked_kp,"
        "n_imu,imu_buf_depth,nav_valid,nav_tracking_state,nav_imu_init,nav_ba2,nav_vel_valid,"
        "map_id,ref_epoch,clock_epoch,kind,navguard_fired,n_ok,n_lost,n_pairs\n");
    enabled_.store(true);
    writer_ = std::thread([this] { writerLoop(); });
    std::cerr << "[TRACKDIAG] enabled -> " << path << std::endl;
}

void TrackDiag::frame(const TrackDiagRow& r)
{
    if (!enabled_.load(std::memory_order_relaxed)) {
        return;
    }
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (buf_.size() >= kMaxBuffered) {
            buf_.pop_front();
            dropped_.fetch_add(1, std::memory_order_relaxed);
        }
        buf_.push_back(r);
    }
}

void TrackDiag::flushLocked(std::vector<TrackDiagRow>& out)
{
    out.clear();
    if (buf_.empty()) {
        return;
    }
    out.reserve(buf_.size());
    while (!buf_.empty()) {
        out.push_back(buf_.front());
        buf_.pop_front();
    }
}

void TrackDiag::writerLoop()
{
    std::vector<TrackDiagRow> batch;
    while (!stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        {
            std::lock_guard<std::mutex> lk(mu_);
            flushLocked(batch);
        }
        if (batch.empty() || !fp_) {
            continue;
        }
        for (const auto& r : batch) {
            std::fprintf(fp_,
                "%llu,%llu,%lld,%lld,%lld,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%llu,%llu,%llu,%d,%d,%d,%d,%d\n",
                (unsigned long long)r.seq, (unsigned long long)r.frame_id,
                (long long)r.src_sim_us, (long long)r.rx_mono_us, (long long)r.submit_epoch_us,
                r.track_ms, r.tracking_state, r.tracked_kp, r.n_imu, r.imu_buf_depth,
                r.nav_valid, r.nav_tracking_state, r.nav_imu_init, r.nav_ba2, r.nav_vel_valid,
                (unsigned long long)r.map_id, (unsigned long long)r.ref_epoch,
                (unsigned long long)r.clock_epoch, r.kind, r.navguard_fired,
                r.n_ok, r.n_lost, r.n_pairs);
        }
        rows_.fetch_add(batch.size(), std::memory_order_relaxed);
        // 单次批量写后 flush：250 ms 一次，不在跟踪线程，也不逐帧。
        std::fflush(fp_);
    }
    // 退出前把剩余行写完
    {
        std::lock_guard<std::mutex> lk(mu_);
        flushLocked(batch);
    }
    if (fp_ && !batch.empty()) {
        for (const auto& r : batch) {
            std::fprintf(fp_,
                "%llu,%llu,%lld,%lld,%lld,%.3f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%llu,%llu,%llu,%d,%d,%d,%d,%d\n",
                (unsigned long long)r.seq, (unsigned long long)r.frame_id,
                (long long)r.src_sim_us, (long long)r.rx_mono_us, (long long)r.submit_epoch_us,
                r.track_ms, r.tracking_state, r.tracked_kp, r.n_imu, r.imu_buf_depth,
                r.nav_valid, r.nav_tracking_state, r.nav_imu_init, r.nav_ba2, r.nav_vel_valid,
                (unsigned long long)r.map_id, (unsigned long long)r.ref_epoch,
                (unsigned long long)r.clock_epoch, r.kind, r.navguard_fired,
                r.n_ok, r.n_lost, r.n_pairs);
        }
        rows_.fetch_add(batch.size(), std::memory_order_relaxed);
    }
}

void TrackDiag::shutdown()
{
    if (!enabled_.exchange(false)) {
        return;
    }
    stop_.store(true);
    if (writer_.joinable()) {
        writer_.join();
    }
    if (fp_) {
        std::fprintf(fp_, "# summary rows=%llu dropped=%llu\n",
                     (unsigned long long)rows_.load(), (unsigned long long)dropped_.load());
        std::fclose(fp_);
        fp_ = nullptr;
    }
    std::cerr << "[TRACKDIAG] closed rows=" << rows_.load() << " dropped=" << dropped_.load() << std::endl;
}

}  // namespace rm27
