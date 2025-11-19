#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

namespace pbpt::utils {

class ProgressBar {
private:
    int m_bar_width{};
    std::size_t m_total_steps{};
    std::string m_description;
    std::chrono::steady_clock::time_point m_start_time{};
    bool m_started{false};
    std::size_t m_completed_steps{0};

    void clear() {
        m_completed_steps = 0;
        m_start_time = std::chrono::steady_clock::now();
        m_started = true;
    }

    static std::string format_duration(double seconds) {
        const double clamped_seconds = seconds < 0.0 ? 0.0 : seconds;
        const auto total_seconds = static_cast<long long>(clamped_seconds + 0.5);
        const auto hours = total_seconds / 3600;
        const auto minutes = (total_seconds % 3600) / 60;
        const auto secs = total_seconds % 60;

        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(2) << hours << ':'
            << std::setfill('0') << std::setw(2) << minutes << ':'
            << std::setfill('0') << std::setw(2) << secs;
        return oss.str();
    }

    static std::string format_rate(std::size_t steps, double seconds) {
        if (steps == 0 || seconds <= 0.0) {
            return "?it/s";
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << (steps / seconds) << "it/s";
        return oss.str();
    }

    std::ostream &render(std::ostream &os) {
        const auto current_step = std::min(m_completed_steps, m_total_steps);
        const double clamped = (
            m_total_steps == 0
                ? 1.0
                : std::clamp(
                      static_cast<double>(current_step) / static_cast<double>(m_total_steps),
                      0.0,
                      1.0
                  )
        );
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_seconds = std::chrono::duration<double>(now - m_start_time).count();
        const int filled = static_cast<int>(clamped * static_cast<double>(m_bar_width));

        os << '\r';
        if (!m_description.empty()) {
            os << m_description << ' ';
        }
        os << std::setw(3) << static_cast<int>(clamped * 100.0 + 0.5) << "%|";
        for (int i = 0; i < m_bar_width; ++i) {
            os << (i < filled ? '=' : ' ');
        }
        os << "| " << current_step << "/" << m_total_steps << " ";

        const auto eta_seconds = clamped > 0.0
            ? elapsed_seconds * (1.0 - clamped) / clamped
            : -1.0;
        os << "[" << format_duration(elapsed_seconds) << "<"
           << (eta_seconds >= 0.0 ? format_duration(eta_seconds) : "??:??:??")
           << ", " << format_rate(current_step, elapsed_seconds) << "]";
        os.flush();
        return os;
    }

public:
    ProgressBar(std::size_t total_steps = 100, int bar_width = 30, std::string description = {})
        : m_bar_width(std::max(10, bar_width)),
          m_total_steps(total_steps == 0 ? 1 : total_steps),
          m_description(std::move(description)) {}

    const std::string& description() const {
        return m_description;
    }

    std::string& description() {
        return m_description;
    }

    const std::size_t& total_steps() const {
        return m_total_steps;
    }

    std::size_t& total_steps() {
        return m_total_steps;
    }

    std::ostream &start(std::ostream &os) {
        clear();
        return render(os);
    }

    std::ostream &update(std::ostream &os, std::size_t steps = 1) {
        if (!m_started) {
            clear();
        }
        if (steps > 0) {
            if (m_completed_steps >= m_total_steps) {
                m_completed_steps = m_total_steps;
            } else {
                const auto remaining = m_total_steps - m_completed_steps;
                m_completed_steps += std::min<std::size_t>(steps, remaining);
            }
        }
        return render(os);
    }

    std::ostream &finish(std::ostream &os) {
        if (!m_started) {
            os << "Progress bar was not started.\n";
            return os;
        }
        m_completed_steps = m_total_steps;
        render(os);
        os << '\n';
        os.flush();
        m_started = false;
        return os;
    }

};

}
