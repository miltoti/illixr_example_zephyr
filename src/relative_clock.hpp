#pragma once

#include <cassert>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <zephyr/sys/atomic.h>   // Zephyr-native atomic — replaces std::atomic

namespace ILLIXR {

/**
 * Mimic of `std::chrono::time_point<Clock, Rep>`.
 */
using _clock_rep      = long;
using _clock_period   = std::nano;
using _clock_duration = std::chrono::duration<_clock_rep, _clock_period>;

class time_point {
public:
    using duration = _clock_duration;

    time_point() = default;

    constexpr explicit time_point(const duration& time_since_epoch)
        : _m_time_since_epoch{time_since_epoch} { }

    [[nodiscard]] duration time_since_epoch() const {
        return _m_time_since_epoch;
    }

    time_point& operator+=(const duration& d) {
        _m_time_since_epoch += d;
        return *this;
    }

    time_point& operator-=(const duration& d) {
        _m_time_since_epoch -= d;
        return *this;
    }

private:
    duration _m_time_since_epoch{};
};

inline time_point::duration operator-(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() - rhs.time_since_epoch();
}
inline time_point operator+(const time_point& pt, const time_point::duration& d) {
    return time_point{pt.time_since_epoch() + d};
}
inline time_point operator+(const time_point::duration& d, const time_point& pt) {
    return time_point{pt.time_since_epoch() + d};
}
inline bool operator<(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() < rhs.time_since_epoch();
}
inline bool operator>(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() > rhs.time_since_epoch();
}
inline bool operator<=(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() <= rhs.time_since_epoch();
}
inline bool operator>=(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() >= rhs.time_since_epoch();
}
inline bool operator==(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() == rhs.time_since_epoch();
}
inline bool operator!=(const time_point& lhs, const time_point& rhs) {
    return lhs.time_since_epoch() != rhs.time_since_epoch();
}

/**
 * @brief Relative clock for all of ILLIXR.
 */
class RelativeClock {
public:
    using rep      = _clock_rep;
    using period   = _clock_period;
    using duration = _clock_duration;
    using tp       = ILLIXR::time_point;

    static constexpr bool is_steady = true;
    static_assert(std::chrono::steady_clock::is_steady);

    RelativeClock() {
        atomic_set(&_dataset_origin_ns, (atomic_val_t)-1);
    }

    [[nodiscard]] tp now() const {
        assert(is_started() && "Cannot call now() before start()");
        auto delta = std::chrono::duration_cast<duration>(
            std::chrono::steady_clock::now() - _m_start);
        return tp{delta};
    }

    [[nodiscard]] std::int64_t now_ns() const {
        return static_cast<std::int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                now().time_since_epoch()).count());
    }

    [[nodiscard]] std::int64_t absolute_ns(tp relative) const {
        auto base_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           _m_start.time_since_epoch()).count();
        auto offset_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             relative.time_since_epoch()).count();
        return static_cast<std::int64_t>(base_ns + offset_ns);
    }

    void start() {
        _m_start  = std::chrono::steady_clock::now();
        _started  = true;
    }

    [[nodiscard]] bool is_started() const { return _started; }

    [[nodiscard]] tp start_time() const {
        auto dur = std::chrono::duration_cast<duration>(_m_start.time_since_epoch());
        return tp{dur};
    }

    void print() const;

    // =========================================================================
    // Dataset time cursor
    // =========================================================================
    //
    // Plugins agree on a shared "dataset origin": the timestamp of the very
    // first sample in the dataset (set once by the first plugin to call
    // set_dataset_origin()).
    //
    // Any plugin can then call dataset_now_ns() to find out which dataset
    // timestamp corresponds to the current wall-clock instant:
    //
    //   dataset_now = dataset_origin + wall_elapsed_since_start
    //
    // This gives all plugins the SAME answer for "what time is it in the
    // dataset?", so they independently decide when to fire without racing.
    //
    // IMU fires at dataset_origin + 0, 5ms, 10ms, ...
    // Camera fires at dataset_origin + 50ms, 100ms, ...
    // Both just compare their next sample's timestamp to dataset_now_ns().
    // =========================================================================

    /**
     * @brief Set the dataset origin (ns). Call once with the timestamp of
     *        the very first sample across all sensors.
     *        Idempotent — only the first call takes effect (CAS on -1).
     */
    void set_dataset_origin(std::int64_t origin_ns) {
        // atomic_cas(ptr, expected, desired) — succeeds only when current == -1.
        // On rv64 atomic_val_t == intptr_t == int64_t, so the cast is safe.
        atomic_cas(&_dataset_origin_ns,
                   (atomic_val_t)-1,
                   (atomic_val_t)origin_ns);
    }

    /**
     * @brief Returns the current dataset timestamp in nanoseconds.
     *
     * dataset_now = dataset_origin + (wall_clock elapsed since start())
     *
     * Returns -1 if the clock hasn't been started or origin hasn't been set.
     */
    [[nodiscard]] std::int64_t dataset_now_ns() const {
        if (!_started) return -1;
        std::int64_t origin = (std::int64_t)atomic_get(&_dataset_origin_ns);
        if (origin < 0) return -1;
        return origin + now_ns();
    }

    /**
     * @brief Convenience: is a sample with this dataset timestamp due yet?
     */
    [[nodiscard]] bool is_due(std::int64_t sample_dataset_ns) const {
        std::int64_t now = dataset_now_ns();
        if (now < 0) return false;
        return now >= sample_dataset_ns;
    }

private:
    std::chrono::steady_clock::time_point _m_start{};
    bool                                  _started{false};

    // atomic_t is Zephyr's RTOS-native atomic integer.
    // On rv64, atomic_val_t == intptr_t == int64_t — exactly what we need.
    // mutable so const methods (dataset_now_ns, is_due) can call atomic_get.
    mutable atomic_t _dataset_origin_ns;
};

using duration = RelativeClock::duration;

template<typename unit = std::ratio<1>>
double duration2double(duration dur) {
    return std::chrono::duration<double, unit>{dur}.count();
}

constexpr duration freq2period(double fps) {
    return duration{
        static_cast<std::size_t>(
            std::chrono::nanoseconds{std::chrono::seconds{1}}.count() / fps)};
}

RelativeClock& get_global_relative_clock();

} // namespace ILLIXR