// [WORKPKG-TRACE][SIM-ONLY] Bounded unified time trace.
//
// Purpose: cover /clock, left/right image and IMU with one consistent set of
// timestamps so the path-latency difference and the Reset chain can be
// analysed OFFLINE without starting extra flights.
//
// Clock domains (explicit):
//   src_sim_us  : Gazebo sim time from the message header (SOURCE time)
//   rmw_recv_us : rmw received_timestamp, host epoch ns->us (may be 0 if the
//                 RMW does not populate it; 0 means "unavailable", NOT zero)
//   cb_wall_us  : SystemMicros() at the very first line of the callback
//   cb_mono_us  : SteadyMicros() at the very first line of the callback
//   submit_*    : enqueue boundary of Bridge::Submit (NOT the publish call)
//
// cb_wall_us is the CALLBACK ENTRY, not the middleware arrival time. The gap
// (cb_wall_us - rmw_recv_us) is executor/queue waiting. Absolute one-way
// transport delay is NOT claimed: with RTF<1 and an unvalidated sim->epoch
// mapping it is not identifiable. Pairing /clock and image at the same (or
// interpolated) source time only yields a PATH DIFFERENCE with an error bound.
#pragma once

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

class TimeTrace {
public:
  struct Rec {
    uint64_t seq = 0;
    uint64_t src_sim_us = 0;
    uint64_t rmw_recv_us = 0;
    uint64_t cb_wall_us = 0;
    uint64_t cb_mono_us = 0;
    uint64_t submit_wall_us = 0;
    uint64_t submit_mono_us = 0;
    int64_t offset_us = 0;
    uint64_t clock_epoch = 0;
    int stream = -1;        // 0=left 1=right 2=imu 3=clock
    int decision = 0;       // 0=pending 1=accept 2=reject 3=drop
    uint32_t reason = 0;    // caller-defined bit flags / counters
  };

  explicit TimeTrace(const std::string &path, size_t cap = 16384)
      : path_(path), cap_(cap), ring_(cap) {
    out_.open(path, std::ios::out | std::ios::trunc);
    if (out_) {
      out_ << "seq,stream,src_sim_us,rmw_recv_us,cb_wall_us,cb_mono_us,"
              "submit_wall_us,submit_mono_us,offset_us,clock_epoch,"
              "decision,reason\n";
    }
    win_.reserve(2 * WIN_HALF + 1);
  }

  // Returns the sequence number of the newly appended record.
  uint64_t Add(int stream, uint64_t src_sim_us, uint64_t rmw_recv_us,
               uint64_t cb_wall_us, uint64_t cb_mono_us) {
    Rec r;
    r.seq = ++seq_;
    r.stream = stream;
    r.src_sim_us = src_sim_us;
    r.rmw_recv_us = rmw_recv_us;
    r.cb_wall_us = cb_wall_us;
    r.cb_mono_us = cb_mono_us;
    if (rmw_recv_us == 0) ++n_rmw_missing_;
    {
      std::lock_guard<std::mutex> lk(mu_);
      ring_[r.seq % cap_] = r;
      ++n_add_;
      if (r.seq % cap_ == 0) ++n_ring_wrap_;
    }
    if (r.seq % FLUSH_EVERY == 0) Flush(false);
    return r.seq;
  }

  void Mark(uint64_t seq, int decision, uint32_t reason,
            uint64_t submit_wall_us, uint64_t submit_mono_us,
            int64_t offset_us, uint64_t clock_epoch) {
    bool first_reject = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (seq > 0 && (seq + cap_) > seq_) {   // still resident in ring
        Rec &r = ring_[seq % cap_];
        if (r.seq == seq) {
          r.decision = decision;
          r.reason = reason;
          r.submit_wall_us = submit_wall_us;
          r.submit_mono_us = submit_mono_us;
          r.offset_us = offset_us;
          r.clock_epoch = clock_epoch;
        } else {
          ++n_lost_mark_;
        }
      } else {
        ++n_lost_mark_;
      }
      if (decision == 2 && first_reject_seq_ == 0) {
        first_reject_seq_ = seq;
        first_reject = true;
      }
      if (decision == 2) ++n_reject_;
      else if (decision == 3) ++n_drop_;
      // Fine-grained window around the FIRST reject: never miss the trigger.
      if ((first_reject_seq_ == 0 ||
           (seq >= first_reject_seq_ - WIN_HALF &&
            seq <= first_reject_seq_ + WIN_HALF)) && win_.size() < 2 * WIN_HALF + 1) {
        if (seq > 0 && (seq + cap_) > seq_) {
          const Rec &r = ring_[seq % cap_];
          if (r.seq == seq) win_.push_back(r);
        }
      }
    }
    if (first_reject) Flush(false);
  }

  // Batched write; never a per-frame flush. Caller may hold no locks.
  void Flush(bool force) {
    std::vector<Rec> snap, wsnap;
    uint64_t upto;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (!force && seq_ - flushed_seq_ < FLUSH_EVERY) return;
      upto = seq_;
      uint64_t from = flushed_seq_ + 1;
      if (from < upto - cap_ + 1) from = upto - cap_ + 1;
      for (uint64_t s = from; s <= upto; ++s) {
        const Rec &r = ring_[s % cap_];
        if (r.seq == s) snap.push_back(r);
      }
      wsnap = win_;
      flushed_seq_ = upto;
      ++n_flush_;
    }
    if (out_) {
      for (const auto &r : snap) WriteLine(r);
      out_.flush();   // one flush per BATCH, not per frame
    }
    if (!wsnap.empty() && force) {
      std::string wp = path_ + ".first_window.csv";
      std::ofstream w(wp, std::ios::out | std::ios::trunc);
      w << "seq,stream,src_sim_us,rmw_recv_us,cb_wall_us,cb_mono_us,"
           "submit_wall_us,submit_mono_us,offset_us,clock_epoch,decision,reason\n";
      for (const auto &r : wsnap) WriteLine(r, w);
    }
  }

  void DumpCounters(std::string *s) const {
    char b[512];
    std::snprintf(b, sizeof(b),
                  "TRACE add=%llu reject=%llu drop=%llu rmw_missing=%llu "
                  "ring_wrap=%llu lost_mark=%llu flushes=%llu first_reject_seq=%llu",
                  (unsigned long long)n_add_, (unsigned long long)n_reject_,
                  (unsigned long long)n_drop_, (unsigned long long)n_rmw_missing_,
                  (unsigned long long)n_ring_wrap_,
                  (unsigned long long)n_lost_mark_, (unsigned long long)n_flush_,
                  (unsigned long long)first_reject_seq_);
    *s = b;
  }

  ~TimeTrace() { Flush(true); }

private:
  static void WriteLine(const Rec &r, std::ostream &o) {
    o << r.seq << ',' << r.stream << ',' << r.src_sim_us << ','
      << r.rmw_recv_us << ',' << r.cb_wall_us << ',' << r.cb_mono_us << ','
      << r.submit_wall_us << ',' << r.submit_mono_us << ',' << r.offset_us << ','
      << r.clock_epoch << ',' << r.decision << ',' << r.reason << '\n';
  }
  void WriteLine(const Rec &r) { WriteLine(r, out_); }

  static constexpr uint64_t FLUSH_EVERY = 2048;
  static constexpr uint64_t WIN_HALF = 256;

  std::string path_;
  size_t cap_;
  std::vector<Rec> ring_;
  std::vector<Rec> win_;
  std::ofstream out_;
  std::mutex mu_;
  std::atomic<uint64_t> seq_{0};
  uint64_t flushed_seq_ = 0;
  uint64_t first_reject_seq_ = 0;
  std::atomic<uint64_t> n_add_{0}, n_reject_{0}, n_drop_{0};
  std::atomic<uint64_t> n_rmw_missing_{0}, n_ring_wrap_{0};
  std::atomic<uint64_t> n_lost_mark_{0}, n_flush_{0};
};
