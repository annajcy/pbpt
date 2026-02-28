#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "pbpt/geometry/spherical.hpp"
#include "pbpt/material/model.hpp"
#include "pbpt/material/plugin/bxdf/dielectric_rough_bxdf.hpp"
#include "pbpt/radiometry/sampled_spectrum.hpp"
#include "pbpt/sampler/3d.hpp"

using namespace pbpt;
using namespace pbpt::geometry;
using namespace pbpt::material;
using namespace pbpt::math;
using namespace pbpt::radiometry;
using namespace pbpt::sampler;

namespace {

template <typename T>
class RandomStream {
public:
    explicit RandomStream(std::uint32_t seed) : m_rng(seed), m_uniform(T(0), T(1)) {}

    T next_1d() { return m_uniform(m_rng); }

    Point<T, 2> next_2d() { return Point<T, 2>(next_1d(), next_1d()); }

private:
    std::mt19937 m_rng;
    std::uniform_real_distribution<T> m_uniform;
};

template <typename T>
Vector<T, 3> normalize_or_throw(const Vector<T, 3>& v) {
    const auto len2 = v.length_squared();
    if (len2 <= T(0)) {
        throw std::runtime_error("cannot normalize zero-length vector");
    }
    return v / std::sqrt(len2);
}

template <typename T, typename Fn>
void for_stratified_uv(int u_count, int v_count, Fn&& fn) {
    for (int v = 0; v < v_count; ++v) {
        for (int u = 0; u < u_count; ++u) {
            const T u_coord = (static_cast<T>(u) + T(0.5)) / static_cast<T>(u_count);
            const T v_coord = (static_cast<T>(v) + T(0.5)) / static_cast<T>(v_count);
            fn(Point<T, 2>(u_coord, v_coord));
        }
    }
}

template <typename T, typename Fn>
void for_stratified_uv_jittered(int u_count, int v_count, RandomStream<T>& rng, Fn&& fn) {
    for (int v = 0; v < v_count; ++v) {
        for (int u = 0; u < u_count; ++u) {
            const T u_coord = (static_cast<T>(u) + rng.next_1d()) / static_cast<T>(u_count);
            const T v_coord = (static_cast<T>(v) + rng.next_1d()) / static_cast<T>(v_count);
            fn(Point<T, 2>(u_coord, v_coord));
        }
    }
}

Vector<double, 3> unit_vector_from_z_phi(double z, double phi) {
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    return Vector<double, 3>(r * std::cos(phi), r * std::sin(phi), z);
}

double integrate_pdf_wm_over_hemisphere(const MicrofacetModel<double>& microfacet, const Vector<double, 3>& wo,
                                       int t_steps, int phi_steps) {
    const double delta_t = 1.0 / static_cast<double>(t_steps);
    const double delta_phi = (2.0 * math::pi_v<double>) / static_cast<double>(phi_steps);

    double integral = 0.0;
    for (int i = 0; i < t_steps; ++i) {
        const double t = (static_cast<double>(i) + 0.5) * delta_t;
        const double z = 1.0 - t * t;
        const double dz = 2.0 * t * delta_t;
        for (int j = 0; j < phi_steps; ++j) {
            const double phi = (static_cast<double>(j) + 0.5) * delta_phi;
            const auto wm = unit_vector_from_z_phi(z, phi);
            integral += static_cast<double>(microfacet.pdf_wm(wo, wm)) * dz * delta_phi;
        }
    }
    return integral;
}

double integrate_pdf_wm_over_bin(const MicrofacetModel<double>& microfacet, const Vector<double, 3>& wo, double z_low,
                                double z_high, double phi_low, double phi_high, int t_steps, int phi_steps) {
    z_low = std::clamp(z_low, 0.0, 1.0);
    z_high = std::clamp(z_high, 0.0, 1.0);
    if (z_high <= z_low) {
        return 0.0;
    }

    auto z_to_t = [](double z) { return std::sqrt(std::max(0.0, 1.0 - z)); };
    const double t_low = z_to_t(z_high);
    const double t_high = z_to_t(z_low);
    const double delta_t = (t_high - t_low) / static_cast<double>(t_steps);
    const double delta_phi = (phi_high - phi_low) / static_cast<double>(phi_steps);

    double integral = 0.0;
    for (int i = 0; i < t_steps; ++i) {
        const double t = t_low + (static_cast<double>(i) + 0.5) * delta_t;
        const double z = 1.0 - t * t;
        const double dz = 2.0 * t * delta_t;
        for (int j = 0; j < phi_steps; ++j) {
            const double phi = phi_low + (static_cast<double>(j) + 0.5) * delta_phi;
            const auto wm = unit_vector_from_z_phi(z, phi);
            integral += static_cast<double>(microfacet.pdf_wm(wo, wm)) * dz * delta_phi;
        }
    }
    return integral;
}

int bin_index_cos_phi(const Vector<double, 3>& wm, int cos_bins, int phi_bins) {
    const double clamped_cos = std::clamp(static_cast<double>(wm.z()), 0.0, 1.0);
    int cos_bin = static_cast<int>(clamped_cos * static_cast<double>(cos_bins));
    cos_bin = std::clamp(cos_bin, 0, cos_bins - 1);

    const double phi_val = static_cast<double>(pbpt::geometry::phi(wm));
    const double phi_normalized = phi_val / (2.0 * math::pi_v<double>);
    int phi_bin = static_cast<int>(phi_normalized * static_cast<double>(phi_bins));
    phi_bin = std::clamp(phi_bin, 0, phi_bins - 1);

    return cos_bin * phi_bins + phi_bin;
}

}  // namespace

TEST(MicrofacetModelTest, GgxNdfNormalizes) {
    using Float = double;
    const std::vector<std::pair<Float, Float>> alpha_pairs = {
        {0.02, 0.18},
        {0.18, 0.02},
        {0.1, 0.1},
    };

    constexpr int u_count = 512;
    constexpr int v_count = 512;
    constexpr std::size_t sample_count = static_cast<std::size_t>(u_count) * static_cast<std::size_t>(v_count);
    const Float uniform_pdf = sample_uniform_hemisphere_pdf<Float>();

    for (std::size_t alpha_index = 0; alpha_index < alpha_pairs.size(); ++alpha_index) {
        const auto [alpha_x, alpha_y] = alpha_pairs[alpha_index];
        MicrofacetModel<Float> microfacet(alpha_x, alpha_y);

        Float estimate = 0;
        RandomStream<Float> rng(1337u + static_cast<std::uint32_t>(alpha_index));
        for_stratified_uv_jittered<Float>(u_count, v_count, rng, [&](const Point<Float, 2>& uv) {
            const auto wm_point = sample_uniform_hemisphere(uv);
            const auto wm = wm_point.to_vector();
            const Float d_val = microfacet.D(wm);
            const Float cos_val = pbpt::geometry::cos_theta(wm);
            estimate += d_val * cos_val / uniform_pdf;
        });
        estimate /= static_cast<Float>(sample_count);

        EXPECT_NEAR(estimate, 1.0, 1e-2) << "NDF normalization failed: alpha_x=" << alpha_x
                                         << " alpha_y=" << alpha_y << " estimate=" << estimate;
    }
}

TEST(MicrofacetModelTest, GgxVndfPdfNormalizes) {
    using Float = double;
    const std::vector<std::pair<Float, Float>> alpha_pairs = {
        {0.02, 0.18},
        {0.18, 0.02},
        {0.1, 0.1},
    };

    const Vector<Float, 3> wo_normal = Vector<Float, 3>(0, 0, 1);
    const Vector<Float, 3> wo_tilt = normalize_or_throw(Vector<Float, 3>(0.6, 0.0, 0.8));
    const Float wo_grazing_x = 0.99;
    const Float wo_grazing_z = std::sqrt(std::max(Float(0), Float(1) - wo_grazing_x * wo_grazing_x));
    const Vector<Float, 3> wo_grazing = normalize_or_throw(Vector<Float, 3>(wo_grazing_x, 0.0, wo_grazing_z));

    const std::vector<Vector<Float, 3>> wo_list = {wo_normal, wo_tilt, wo_grazing};

    constexpr int t_steps = 1024;
    constexpr int phi_steps = 1024;

    for (std::size_t alpha_index = 0; alpha_index < alpha_pairs.size(); ++alpha_index) {
        const auto [alpha_x, alpha_y] = alpha_pairs[alpha_index];
        MicrofacetModel<Float> microfacet(alpha_x, alpha_y);

        for (std::size_t wo_index = 0; wo_index < wo_list.size(); ++wo_index) {
            const auto wo = wo_list[wo_index];
            ASSERT_GT(wo.z(), 0.0);

            const Float estimate = integrate_pdf_wm_over_hemisphere(microfacet, wo, t_steps, phi_steps);

            EXPECT_NEAR(estimate, 1.0, 1e-2) << "VNDF pdf normalization failed: alpha_x=" << alpha_x
                                             << " alpha_y=" << alpha_y << " wo=(" << wo.x() << "," << wo.y() << ","
                                             << wo.z() << ") estimate=" << estimate;
        }
    }
}

TEST(MicrofacetModelTest, GgxVndfPdfNormalizesForNegativeWo) {
    using Float = double;
    const std::vector<std::pair<Float, Float>> alpha_pairs = {
        {0.02, 0.18},
        {0.18, 0.02},
        {0.1, 0.1},
    };

    const Vector<Float, 3> wo_inside = Vector<Float, 3>(0, 0, -1);
    constexpr int t_steps = 1024;
    constexpr int phi_steps = 1024;

    for (const auto& [alpha_x, alpha_y] : alpha_pairs) {
        MicrofacetModel<Float> microfacet(alpha_x, alpha_y);
        const Float estimate = integrate_pdf_wm_over_hemisphere(microfacet, wo_inside, t_steps, phi_steps);
        EXPECT_NEAR(estimate, 1.0, 1e-2) << "VNDF pdf normalization failed for negative wo: alpha_x=" << alpha_x
                                         << " alpha_y=" << alpha_y << " estimate=" << estimate;
    }
}

TEST(MicrofacetModelTest, SampleWmMatchesPdfWmHistogram) {
    using Float = double;

    constexpr int cos_bins = 12;
    constexpr int phi_bins = 24;
    constexpr int bin_count = cos_bins * phi_bins;

    constexpr int u_count = 512;
    constexpr int v_count = 512;
    constexpr std::size_t observed_sample_count = static_cast<std::size_t>(u_count) * static_cast<std::size_t>(v_count);

    const MicrofacetModel<Float> microfacet(0.02, 0.18);

    const std::vector<Vector<Float, 3>> wo_list = {
        Vector<Float, 3>(0, 0, 1),
        normalize_or_throw(Vector<Float, 3>(0.6, 0.0, 0.8)),
    };

    constexpr int t_steps = 64;
    constexpr int phi_steps = 32;

    for (std::size_t wo_index = 0; wo_index < wo_list.size(); ++wo_index) {
        const auto wo = wo_list[wo_index];
        ASSERT_GT(wo.z(), 0.0);

        std::vector<double> expected_probability(bin_count, 0.0);
        for (int cos_bin = 0; cos_bin < cos_bins; ++cos_bin) {
            const double z_low = static_cast<double>(cos_bin) / static_cast<double>(cos_bins);
            const double z_high = static_cast<double>(cos_bin + 1) / static_cast<double>(cos_bins);
            for (int phi_bin = 0; phi_bin < phi_bins; ++phi_bin) {
                const double phi_low = (static_cast<double>(phi_bin) / static_cast<double>(phi_bins)) *
                                       (2.0 * math::pi_v<double>);
                const double phi_high = (static_cast<double>(phi_bin + 1) / static_cast<double>(phi_bins)) *
                                        (2.0 * math::pi_v<double>);
                const double expected_prob =
                    integrate_pdf_wm_over_bin(microfacet, wo, z_low, z_high, phi_low, phi_high, t_steps, phi_steps);
                expected_probability[static_cast<std::size_t>(cos_bin * phi_bins + phi_bin)] = expected_prob;
            }
        }

        std::vector<std::size_t> observed_count(bin_count, 0);
        RandomStream<Float> observed_rng(42u + static_cast<std::uint32_t>(wo_index));
        for_stratified_uv_jittered<Float>(u_count, v_count, observed_rng, [&](const Point<Float, 2>& uv) {
            const auto wm = microfacet.sample_wm(wo, uv);
            const int idx = bin_index_cos_phi(wm, cos_bins, phi_bins);
            observed_count[static_cast<std::size_t>(idx)] += 1;
        });

        double expected_sum = 0.0;
        for (const auto value : expected_probability) {
            expected_sum += value;
        }
        EXPECT_NEAR(expected_sum, 1.0, 2e-2) << "Expected bin probabilities should sum to ~1; got " << expected_sum;

        constexpr double probability_threshold = 5e-4;
        for (int idx = 0; idx < bin_count; ++idx) {
            const double expected_prob = expected_probability[static_cast<std::size_t>(idx)];
            if (expected_prob < probability_threshold) {
                continue;
            }
            const double observed_prob =
                static_cast<double>(observed_count[static_cast<std::size_t>(idx)]) / static_cast<double>(observed_sample_count);
            const double rel_error = std::abs(observed_prob - expected_prob) / expected_prob;
            EXPECT_LT(rel_error, 0.15) << "Histogram mismatch: wo_index=" << wo_index << " idx=" << idx
                                       << " expected=" << expected_prob << " observed=" << observed_prob
                                       << " rel_error=" << rel_error;
        }
    }
}

TEST(MicrofacetModelTest, DielectricRoughSamplingMatchesUniform) {
    using Float = double;
    constexpr int N = 4;

    const MicrofacetModel<Float> microfacet(0.02, 0.18);
    DielectricRoughBxDF<Float, N> bxdf(1.5, microfacet);

    SampledWavelength<Float, N> swl{};
    for (int i = 0; i < N; ++i) {
        swl[i] = 500.0;
    }

    const std::vector<Vector<Float, 3>> wo_list = {
        Vector<Float, 3>(0, 0, 1),
        normalize_or_throw(Vector<Float, 3>(0.6, 0.0, 0.8)),
    };

    constexpr int is_u_count = 512;
    constexpr int is_v_count = 512;
    constexpr std::size_t is_sample_count =
        static_cast<std::size_t>(is_u_count) * static_cast<std::size_t>(is_v_count);

    constexpr int uniform_u_count = 1024;
    constexpr int uniform_v_count = 1024;
    constexpr std::size_t uniform_sample_count =
        static_cast<std::size_t>(uniform_u_count) * static_cast<std::size_t>(uniform_v_count);

    std::vector<Float> uc(is_sample_count);
    std::vector<Point<Float, 2>> u2d(is_sample_count);
    RandomStream<Float> is_rng(1337u);
    for (std::size_t i = 0; i < is_sample_count; ++i) {
        uc[i] = (static_cast<Float>(i) + is_rng.next_1d()) / static_cast<Float>(is_sample_count);
    }
    std::size_t idx = 0;
    for_stratified_uv_jittered<Float>(is_u_count, is_v_count, is_rng, [&](const Point<Float, 2>& uv) {
        u2d[idx] = uv;
        idx += 1;
    });

    const Float uniform_pdf = sample_uniform_sphere_pdf<Float>();

    for (std::size_t wo_index = 0; wo_index < wo_list.size(); ++wo_index) {
        const auto wo = wo_list[wo_index];
        ASSERT_GT(wo.z(), 0.0);

        const auto is_estimate_spectrum = bxdf.rou_hd(swl, wo, uc, u2d, TransportMode::Radiance);
        const Float is_estimate = is_estimate_spectrum[0];

        Float uniform_estimate = 0;
        RandomStream<Float> uniform_rng(20260227u + static_cast<std::uint32_t>(wo_index));
        for_stratified_uv_jittered<Float>(uniform_u_count, uniform_v_count, uniform_rng, [&](const Point<Float, 2>& uv) {
            const auto wi_point = sample_uniform_sphere(uv);
            const auto wi = wi_point.to_vector();
            const auto f_val = bxdf.f(swl, wo, wi, TransportMode::Radiance);
            const Float abs_cos = std::abs(pbpt::geometry::cos_theta(wi));
            uniform_estimate += f_val[0] * abs_cos / uniform_pdf;
        });
        uniform_estimate /= static_cast<Float>(uniform_sample_count);

        const Float denom = std::max(Float(1e-6), std::abs(uniform_estimate));
        const Float rel_error = std::abs(is_estimate - uniform_estimate) / denom;
        EXPECT_LT(rel_error, 0.03) << "IS vs Uniform mismatch: wo=(" << wo.x() << "," << wo.y() << "," << wo.z()
                                   << ") is=" << is_estimate << " uniform=" << uniform_estimate
                                   << " rel_error=" << rel_error;

        // Spot-check sample_f consistency with pdf() and f() on a small subset.
        constexpr std::size_t spot_check_count = 1000;
        for (std::size_t i = 0; i < spot_check_count; ++i) {
            const auto sample_opt = bxdf.sample_f(swl, wo, uc[i], u2d[i], TransportMode::Radiance);
            if (!sample_opt) {
                continue;
            }
            const auto& sample = *sample_opt;
            const Float pdf_eval = bxdf.pdf(wo, sample.wi, TransportMode::Radiance);
            EXPECT_NEAR(sample.pdf, pdf_eval, 1e-6) << "pdf mismatch at i=" << i << " sample.pdf=" << sample.pdf
                                                    << " pdf_eval=" << pdf_eval;

            const auto f_eval = bxdf.f(swl, wo, sample.wi, TransportMode::Radiance);
            EXPECT_NEAR(sample.f[0], f_eval[0], 1e-6) << "f mismatch at i=" << i << " sample.f=" << sample.f[0]
                                                      << " f_eval=" << f_eval[0];
        }
    }
}
