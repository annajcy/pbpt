#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "radiometry/spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Spectrum defined by piecewise linear interpolation of sample points.
 *
 * Sample points are (wavelength, value) pairs; the spectrum value
 * is linearly interpolated between neighboring points.
 */
template<typename T>
class PiecewiseLinearSpectrumDistribution : public SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T> {
    friend class SpectrumDistribution<PiecewiseLinearSpectrumDistribution<T>, T>;
public:
    static PiecewiseLinearSpectrumDistribution from_string(const std::string& str) {
        std::vector<std::pair<T, T>> points;
        for (int i = 0, c = 0; i < str.size();) {
            while (i < str.size() && std::isspace(str[i])) i++;
            for (c = i; c < str.size() && str[c] != ','; c++);
            std::string point_str = str.substr(i, c - i);
            size_t sep = point_str.find(':');
            if (sep == std::string::npos) {
                throw std::invalid_argument("Invalid point format in PiecewiseLinearSpectrumDistribution");
            }
            T lambda = static_cast<T>(std::atof(point_str.substr(0, sep).c_str()));
            T value  = static_cast<T>(std::atof(point_str.substr(sep + 1).c_str()));
            points.emplace_back(lambda, value);
            i = ++c;
        }
        return PiecewiseLinearSpectrumDistribution(points);
    }

private:
    std::vector<std::pair<T, T>> m_points; // (lambda, value)

public:
    /**
     * @brief Spectrum defined by a set of (wavelength, value) knots.
     *
     * Values between knots are obtained by linear interpolation.
     */
    PiecewiseLinearSpectrumDistribution(const std::vector<std::pair<T, T>>& points)
        : m_points(points) {
            sort_points();
        }

private:
    constexpr T at_impl(T lambda) const {
        if (m_points.empty()) {
            return T(0);
        }

        if (lambda <= m_points.front().first) {
            return m_points.front().second;
        }

        if (lambda >= m_points.back().first) {
            return m_points.back().second;
        }

        auto it = std::lower_bound(
            m_points.begin(), 
            m_points.end(), 
            lambda,
            [](const auto& point, T value) { return point.first < value; }
        );

        const auto& [lambda1, value1] = *it;
        const auto& [lambda0, value0] = *(it - 1);
        
        return value0 + (value1 - value0) * (lambda - lambda0) / (lambda1 - lambda0);
    }

    void sort_points() {
        std::sort(m_points.begin(), m_points.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }

    /**
     * @brief Adds a new knot to the piecewise linear spectrum.
     *
     * The new point is inserted without re-sorting; call sort_points()
     * afterwards if you rely on the ordering of the knot list.
     */
    void add_point(T lambda, T value) {
        m_points.emplace_back(lambda, value);
    }

    /**
     * @brief Removes all knots with the given wavelength.
     *
     * If multiple points share the same wavelength, they are all removed.
     */
    void remove_point(T lambda) {
        std::erase_if(m_points, [lambda](const auto& p) {
            return p.first == lambda;
        });
    }
};

}  // namespace pbpt::radiometry
