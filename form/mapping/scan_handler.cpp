#include "form/mapping/scan_handler.hpp"

namespace form {

// ------------------------- Doers ------------------------- //
std::vector<FrameIndex>
ScanHandler::update(FrameIndex idx, size_t size,
                    std::function<size_t(FrameIndex)> connections) noexcept {
  if (idx == 0) {
    m_keyframes.push_back(Frame(idx, size));
  } else {
    m_recent_frames.push_back(Frame(idx, size));
  }

  std::vector<FrameIndex> marg_results;

  // First we handle recent frames
  if (m_recent_frames.size() > m_params.max_num_recent_frames) {
    const auto rf = m_recent_frames.front();
    m_recent_frames.pop_front();

    // If it's a keyframe, add it to the keyframe indices
    auto ratio = static_cast<double>(connections(rf.idx)) /
                 (static_cast<double>(rf.size * m_recent_frames.size()));

    if (ratio > m_params.keyscan_match_ratio) {
      m_keyframes.push_back(rf);
    }
    // If not, we marginalize it out
    else {
      marg_results.push_back(rf.idx);
    }
  }

  std::set<FrameIndex> finished_keyframes;
  for (auto &kf : m_keyframes) {
    // If this keyframe is connected to a recent frame, we don't marginalize it
    if (connections(kf.idx) > 0) {
      kf.unused_count = 0;
    } else {
      ++kf.unused_count;
    }

    // If this keyframe has been unused for too long, we marginalize it out
    if (kf.unused_count > m_params.max_steps_unused_keyframe) {
      marg_results.push_back(kf.idx);
      finished_keyframes.insert(kf.idx);
    }
  }

  m_keyframes.erase(std::remove_if(m_keyframes.begin(), m_keyframes.end(),
                                   [&](const Frame &f) {
                                     return finished_keyframes.find(f.idx) !=
                                            finished_keyframes.end();
                                   }),
                    m_keyframes.end());

  // Marginalize keyframe we have too many keyframes
  // This should ideally never be reached in actuality
  if ((m_params.max_num_keyframes > 0 &&
       m_keyframes.size() > m_params.max_num_keyframes)) {
    const auto kf = m_keyframes.front();
    m_keyframes.pop_front();
    marg_results.push_back(kf.idx);
  }

  return marg_results;
}

const void ScanHandler::fill_constraints(
    tsl::robin_map<FrameIndex,
                   std::tuple<feature::PlanePoint::Ptr, feature::PointPoint::Ptr>>
        &constraints) noexcept {
  // Add empty vectors for all frames
  for (const auto &frame_i : m_keyframes) {
    constraints.insert(std::make_pair(
        frame_i.idx, std::make_tuple(std::make_shared<feature::PlanePoint>(),
                                     std::make_shared<feature::PointPoint>())));
  }
  for (const auto &frame_i : m_recent_frames) {
    constraints.insert(std::make_pair(
        frame_i.idx, std::make_tuple(std::make_shared<feature::PlanePoint>(),
                                     std::make_shared<feature::PointPoint>())));
  }
}

} // namespace form