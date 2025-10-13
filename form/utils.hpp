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
#pragma once

#include <chrono>
#include <cstdio>
#include <optional>
#include <string>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <tuple>

namespace form {

namespace tuple {
// Some helpers ot make iterating over tuples easier
// https://www.cppstories.com/2022/tuple-iteration-apply/

/// @brief Transform each element of a tuple, returning a new tuple
template <typename TupleT, typename Fn>
[[nodiscard]] auto transform(TupleT &&tp, Fn &&fn) {
  return std::apply(
      [&fn](auto &&...args) {
        return std::make_tuple(fn(std::forward<decltype(args)>(args))...);
      },
      std::forward<TupleT>(tp));
}

/// @brief Iterate over each element of a tuple
template <typename TupleT, typename Fn> void for_each(TupleT &&tp, Fn &&fn) {
  std::apply(
      [&fn](auto &&...args) { (fn(std::forward<decltype(args)>(args)), ...); },
      std::forward<TupleT>(tp));
}

/// @brief Iterate over a std::integer_sequence, useful for iterating over tuples
template <typename T, T... S, typename F>
constexpr void for_seq(std::integer_sequence<T, S...>, F &&f) {
  (void(f(std::integral_constant<T, S>{})), ...);
}

} // namespace tuple

/// @brief Set the number of threads for TBB
/// @param num_threads Number of threads to use. If 0, use the maximum number
/// of threads available.
/// @return A tbb::global_control object that sets the number of threads for
/// the current scope.
/// @note The returned object must be kept alive for the duration of the scope
/// usually with a static const in a constructor
inline tbb::global_control set_num_threads(size_t num_threads) {
  size_t max_num_threads =
      num_threads > 0 ? num_threads : tbb::this_task_arena::max_concurrency();
  return tbb::global_control(tbb::global_control::max_allowed_parallelism,
                             max_num_threads);
}

/// @brief A simple timer class to measure elapsed time
class Timer {
public:
  Timer() : m_start(std::chrono::high_resolution_clock::now()) {}

  void reset() { m_start = std::chrono::high_resolution_clock::now(); }

  static void print(double duration, std::string name) {
    std::printf("%s: %f ms\n", name.c_str(), duration);
  }

  double elapsed(std::optional<std::string> name = std::nullopt) {
    using namespace std::chrono;
    auto end = high_resolution_clock::now();
    duration<double, std::milli> diff = end - m_start;
    if (name) {
      print(diff.count(), *name);
    }
    reset();
    return diff.count();
  }

private:
  std::chrono::high_resolution_clock::time_point m_start;
};

} // namespace form
