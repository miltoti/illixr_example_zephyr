#pragma once

#include <cassert>
#include <chrono>
#include <cstdint>
#include <ratio>

namespace ILLIXR {

/**
 * Mimic of `std::chrono::time_point<Clock, Rep>`.
 *
 * We can't use std::chrono::time_point<RelativeClock> directly because
 * RelativeClock::now() is stateful (instance method), not a pure static.
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

inline time_point::duration operator-(const time_point& lhs,
                                      const time_point& rhs) {
    return lhs.time_since_epoch() - rhs.time_since_epoch();
}

inline time_point operator+(const time_point& pt,
                            const time_point::duration& d) {
    return time_point{pt.time_since_epoch() + d};
}

inline time_point operator+(const time_point::duration& d,
                            const time_point& pt) {
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
 *
 * Zephyr port version:
 *  - Uses std::chrono::steady_clock under the hood.
 *  - Provides relative time since start().
 *  - Uses the custom ILLIXR::time_point above.
 *  - Access via get_global_relative_clock().
 */
class RelativeClock {
public:
    using rep        = _clock_rep;
    using period     = _clock_period;
    using duration   = _clock_duration;
    using tp         = ILLIXR::time_point;  // avoid name clash with class name

    static constexpr bool is_steady = true;
    static_assert(std::chrono::steady_clock::is_steady);

    RelativeClock() = default;

    /**
     * @brief Current relative time since start().
     *
     * Requires the clock to be started; otherwise asserts.
     */
    [[nodiscard]] tp now() const {
        assert(is_started() && "Cannot call now() before start()");
        auto delta = std::chrono::duration_cast<duration>(
            std::chrono::steady_clock::now() - _m_start);
        return tp{delta};
    }

    /**
     * @brief Current relative time in nanoseconds since start().
     */
    [[nodiscard]] std::int64_t now_ns() const {
        auto rel = now();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      rel.time_since_epoch())
                      .count();
        return static_cast<std::int64_t>(ns);
    }

    /**
     * @brief Convert a relative time_point to an absolute ns timestamp
     *        (steady_clock epoch + relative offset).
     */
    [[nodiscard]] std::int64_t absolute_ns(tp relative) const {
        auto base_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           _m_start.time_since_epoch())
                           .count();
        auto offset_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                relative.time_since_epoch())
                .count();
        return static_cast<std::int64_t>(base_ns + offset_ns);
    }

    /**
     * @brief Starts the clock. All times are relative to this point.
     */
    void start() {
        _m_start  = std::chrono::steady_clock::now();
        _started  = true;
    }

    /**
     * @brief Check if the clock is started.
     */
    [[nodiscard]] bool is_started() const {
        return _started;
    }

    /**
     * @brief Get the start time of the clock in steady_clock domain.
     *
     * This is NOT 0; it’s the duration since steady_clock epoch at start().
     */
    [[nodiscard]] tp start_time() const {
        auto dur = std::chrono::duration_cast<duration>(
            _m_start.time_since_epoch());
        return tp{dur};
    }

    /**
     * @brief Debug print of the current relative time in ns.
     */
    void print() const;

private:
    std::chrono::steady_clock::time_point _m_start{};
    bool                                  _started{false};
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

/**
 * @brief Global singleton accessor used by the Zephyr runtime.
 */
RelativeClock& get_global_relative_clock();

} // namespace ILLIXR
