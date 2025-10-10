#include "tsl/robin_map.h"
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <vector>

#include "form/feature/factor.hpp"

namespace form {

using FrameIndex = size_t;

struct Frame {
  size_t idx;
  size_t unused_count = 0;
  size_t size = 0;

  Frame(size_t idx_, size_t size_ = 0) : idx(idx_), size(size_) {}
};

class KeyScanner {
public:
  struct Params {
    // Maximum number of keyframes to keep
    int64_t max_num_keyframes = 50;
    // Maximum number of steps a keyframe can go unused before being removed
    int64_t max_steps_unused_keyframe = 10;
    // Maximum number of recent frames to keep
    size_t max_num_recent_frames = 10;
    // Keyscan matching ratio
    double keyscan_match_ratio = 0.1;
  };

private:
  // Parameters
  Params m_params;

  // Recent frames
  std::deque<Frame> m_recent_frames;

  // Key frames
  std::deque<Frame> m_keyframes;

public:
  KeyScanner() : m_params() {}
  KeyScanner(const Params &params) : m_params(params) {}

  // ------------------------- Doers ------------------------- //
  std::vector<FrameIndex>
  update(FrameIndex idx, size_t size,
         std::function<size_t(FrameIndex)> connections) noexcept;

  const void fill_constraints(
      tsl::robin_map<FrameIndex, std::tuple<PlanePoint::Ptr, PointPoint::Ptr>>
          &constraints) noexcept;

  // ------------------------- Getters ------------------------- //
  const size_t size() const noexcept {
    return m_keyframes.size() + m_recent_frames.size();
  }
  const size_t newest_rf() const noexcept { return m_recent_frames.back().idx; }
  const size_t oldest_rf() const noexcept { return m_recent_frames.front().idx; }
  const size_t newest_kf() const noexcept { return m_keyframes.back().idx; }
  const size_t oldest_kf() const noexcept { return m_keyframes.front().idx; }

  const std::deque<Frame> &all_rf() const noexcept { return m_recent_frames; }
  const std::deque<Frame> &all_kf() const noexcept { return m_keyframes; }
};

} // namespace form