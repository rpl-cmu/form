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
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <vector>

namespace form {

using FrameIndex = size_t;

/// @brief Scan representation
struct Scan {
  /// @brief Index of the frame
  size_t idx;

  /// @brief Number of times the frame has gone unconnected to recent frames
  /// (only used if scan becomes a keyframe)
  size_t unused_count = 0;

  /// @brief Size of the frame (number of features)
  size_t size = 0;

  /// @brief Constructor
  Scan(size_t idx_, size_t size_ = 0) : idx(idx_), size(size_) {}
};

/// @brief Keyscan manager
///
/// Maintains a set of recent frames and keyframes, adding/removing them based on
/// connectivity to recent frames
class KeyScanner {
public:
  /// @brief Parameters for the keyscanner
  struct Params {
    /// @brief Maximum number of keyframes to keep
    int64_t max_num_keyframes = 50;
    /// @brief Maximum number of steps a keyframe can go unused before being removed
    int64_t max_steps_unused_keyframe = 10;
    /// @brief Maximum number of recent frames to keep
    size_t max_num_recent_frames = 10;
    /// @brief Keyscan matching ratio
    double keyscan_match_ratio = 0.1;
  };

private:
  /// @brief Parameters
  Params m_params;

  /// @brief Recent frames
  std::deque<Scan> m_recent_frames;

  /// @brief Key frames
  std::deque<Scan> m_keyframes;

public:
  /// @brief Default constructor
  KeyScanner() : m_params() {}

  /// @brief Parameterized constructor
  KeyScanner(const Params &params) : m_params(params) {}

  // ------------------------- Doers ------------------------- //
  /// @brief Update the keyscanner with a new frame.
  ///
  /// Will perform all the transitions from recent frame to keyframe and
  /// marginalization checks.
  ///
  /// @param idx Index of the new frame
  /// @param size Size of the new frame (number of features)
  /// @param connections Function that returns the number of connections a frame
  /// has to recent frames
  /// @return Vector of frame indices that need to be marginalized out
  std::vector<FrameIndex>
  step(FrameIndex idx, size_t size,
       std::function<size_t(FrameIndex)> connections) noexcept;

  // ------------------------- Getters ------------------------- //
  /// @brief Get the number of frames being tracked
  const size_t size() const noexcept {
    return m_keyframes.size() + m_recent_frames.size();
  }
  /// @brief Get newest recent frame index
  const size_t newest_rf() const noexcept { return m_recent_frames.back().idx; }

  /// @brief Get oldest recent frame index
  const size_t oldest_rf() const noexcept { return m_recent_frames.front().idx; }

  /// @brief Get newest keyframe index
  const size_t newest_kf() const noexcept { return m_keyframes.back().idx; }

  /// @brief Get oldest keyframe index
  const size_t oldest_kf() const noexcept { return m_keyframes.front().idx; }
};

} // namespace form