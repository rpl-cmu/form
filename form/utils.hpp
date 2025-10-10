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
template <typename TupleT, typename Fn>
[[nodiscard]] auto transform(TupleT &&tp, Fn &&fn) {
  return std::apply(
      [&fn](auto &&...args) {
        return std::make_tuple(fn(std::forward<decltype(args)>(args))...);
      },
      std::forward<TupleT>(tp));
}

template <typename TupleT, typename Fn> void for_each(TupleT &&tp, Fn &&fn) {
  std::apply(
      [&fn](auto &&...args) { (fn(std::forward<decltype(args)>(args)), ...); },
      std::forward<TupleT>(tp));
}

template <typename T, T... S, typename F>
constexpr void for_seq(std::integer_sequence<T, S...>, F &&f) {
  (void(f(std::integral_constant<T, S>{})), ...);
}

} // namespace tuple

inline tbb::global_control set_num_threads(size_t num_threads) {
  size_t max_num_threads =
      num_threads > 0 ? num_threads : tbb::this_task_arena::max_concurrency();
  return tbb::global_control(tbb::global_control::max_allowed_parallelism,
                             max_num_threads);
}

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
