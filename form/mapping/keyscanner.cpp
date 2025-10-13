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
std::vector<ScanIndex>
KeyScanner::step(ScanIndex idx, size_t size,
                 std::function<size_t(ScanIndex)> connections) noexcept {
  if (idx == 0) {
    m_keyscans.push_back(Scan(idx, size));
  } else {
    m_recent_scans.push_back(Scan(idx, size));
  }

  std::vector<ScanIndex> marg_results;

  // First we handle recent scans
  if (m_recent_scans.size() > m_params.max_num_recent_scans) {
    const auto rf = m_recent_scans.front();
    m_recent_scans.pop_front();

    // If it's a keyscan, add it to the keyscan indices
    auto ratio = static_cast<double>(connections(rf.idx)) /
                 (static_cast<double>(rf.size * m_recent_scans.size()));

    if (ratio > m_params.keyscan_match_ratio) {
      m_keyscans.push_back(rf);
    }
    // If not, we marginalize it out
    else {
      marg_results.push_back(rf.idx);
    }
  }

  std::set<ScanIndex> finished_keyscans;
  for (auto &kf : m_keyscans) {
    // If this keyscan is connected to a recent scan, we don't marginalize it
    if (connections(kf.idx) > 0) {
      kf.unused_count = 0;
    } else {
      ++kf.unused_count;
    }

    // If this keyscan has been unused for too long, we marginalize it out
    if (kf.unused_count > m_params.max_steps_unused_keyscan) {
      marg_results.push_back(kf.idx);
      finished_keyscans.insert(kf.idx);
    }
  }

  m_keyscans.erase(std::remove_if(m_keyscans.begin(), m_keyscans.end(),
                                  [&](const Scan &f) {
                                    return finished_keyscans.find(f.idx) !=
                                           finished_keyscans.end();
                                  }),
                   m_keyscans.end());

  // Marginalize keyscan we have too many keyscans
  // This should ideally never be reached in actuality
  if ((m_params.max_num_keyscans > 0 &&
       m_keyscans.size() > m_params.max_num_keyscans)) {
    const auto kf = m_keyscans.front();
    m_keyscans.pop_front();
    marg_results.push_back(kf.idx);
  }

  return marg_results;
}

} // namespace form