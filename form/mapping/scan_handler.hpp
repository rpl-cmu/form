#include "tsl/robin_map.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <set>
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

class ScanHandler {
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
  ScanHandler() : m_params() {};
  ScanHandler(const Params &params) : m_params(params) {}

  // ------------------------- Doers ------------------------- //
  std::vector<FrameIndex>
  update(FrameIndex idx, size_t size,
         std::function<size_t(FrameIndex)> connections) noexcept;

  const void fill_constraints(
      tsl::robin_map<FrameIndex,
                     std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>>
          &constraints) noexcept;

  // ------------------------- Getters ------------------------- //
  const Frame newest_rf() const noexcept { return m_recent_frames.back(); }
  const Frame oldest_rf() const noexcept { return m_recent_frames.front(); }
  const Frame newest_kf() const noexcept { return m_keyframes.back(); }
  const Frame oldest_kf() const noexcept { return m_keyframes.front(); }

  const std::deque<Frame> &all_rf() const noexcept { return m_recent_frames; }
  const std::deque<Frame> &all_kf() const noexcept { return m_keyframes; }
};

} // namespace form