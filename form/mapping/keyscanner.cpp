// MIT License

// Copyright (c) 2025 Easton Potokar, Taylor Pool, and Michael Kaess

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "form/mapping/keyscanner.hpp"
#include <algorithm>
#include <set>

namespace form {

// ------------------------- Doers ------------------------- //
std::vector<FrameIndex>
KeyScanner::step(FrameIndex idx, size_t size,
                 std::function<size_t(FrameIndex)> connections) noexcept {
  if (idx == 0) {
    m_keyframes.push_back(Scan(idx, size));
  } else {
    m_recent_frames.push_back(Scan(idx, size));
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
                                   [&](const Scan &f) {
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

} // namespace form