/**
 * @file
 * @brief Helpers for loading spectral data from CSV files.
 */
#pragma once

#include <array>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <vector>

#include "spectrum_distribution.hpp"

namespace pbpt::radiometry {

/**
 * @brief Simple CSV reader for spectral data.
 *
 * Assumes each non-comment line has the form:
 *   wavelength, value0, value1, ...
 * and returns a list of (wavelength, values) pairs.
 *
 * @tparam T Scalar type of the loaded values.
 */
template<typename T>
class CsvSpectrumDataSource {
private:
    std::string file_path_;
    char delimiter_;
    
public:
    /**
     * @brief Constructs a CSV spectrum data source.
     *
     * @param path      Path to the CSV file on disk.
     * @param delimiter Field delimiter character (default comma).
     */
    explicit CsvSpectrumDataSource(const std::string& path, char delimiter = ',') 
        : file_path_(path), delimiter_(delimiter) {}
    
    /**
     * @brief Loads all spectral samples from the CSV file.
     *
     * Comment lines starting with '#' and empty lines are skipped.
     * Each remaining line is parsed as an integer wavelength followed
     * by a variable number of scalar values. The function returns a
     * vector of pairs (wavelength, list of values).
     *
     * @throws std::runtime_error if the file cannot be opened.
     */
    std::vector<std::pair<int, std::vector<T>>> load_data() {
        std::ifstream fin(file_path_);
        if (!fin) throw std::runtime_error("Cannot open CSV: " + file_path_);
        
        std::vector<std::pair<int, std::vector<T>>> data;
        std::string line;
        
        while (std::getline(fin, line)) {
            if (should_skip_line(line)) continue;
            
            auto parsed = parse_line(line);
            if (parsed.first >= 0) { // valid wavelength
                data.push_back(parsed);
            }
        }
        
        return data;
    }
    
private:
    bool should_skip_line(const std::string& line) const {
        return line.empty() || line[0] == '#' || 
               line.find_first_of("0123456789") == std::string::npos;
    }
    
    std::pair<int, std::vector<T>> parse_line(const std::string& line) const {
        std::string processed_line = line;
        std::replace(processed_line.begin(), processed_line.end(), delimiter_, ' ');
        
        std::istringstream iss(processed_line);
        int wavelength;
        if (!(iss >> wavelength)) return {-1, {}};
        
        std::vector<T> values;
        T value;
        while (iss >> value) {
            values.push_back(value);
        }
        
        return {wavelength, values};
    }
};

/**
 * @brief Create a set of tabular spectra from a CSV file.
 *
 * The CSV is read using @c CsvSpectrumDataSource, and the first
 * Range::Count rows are used to fill @p spectrum_count separate
 * tabular spectra, each corresponding to one column of values.
 *
 * @tparam T     Scalar type.
 * @tparam Range TabularSpectrumRange describing wavelength bounds.
 * @param csv_path       Path to the CSV file.
 * @param spectrum_count Number of spectra (columns) to create.
 * @return Vector of @c TabularSpectrumDistribution instances.
 */
template<typename T, typename Range>
inline auto make_spectra_from_csv(const std::string& csv_path, int spectrum_count) {

    auto source = CsvSpectrumDataSource<T>(csv_path);
    auto data = source.load_data();

    std::vector<std::array<T, Range::Count>> spectra_data(spectrum_count);

    for (int i = 0; i < Range::Count; ++i) {
        auto &[wavelength, values] = data[i];
        for (int j = 0; j < spectrum_count && j < static_cast<int>(values.size()); ++j) {
            spectra_data[j][i] = values[j];
        }
    }

    std::vector<radiometry::TabularSpectrumDistribution<T, Range::LMinValue, Range::LMaxValue>> result{};
    for (int i = 0; i < spectrum_count; ++i) {
        result.push_back(radiometry::TabularSpectrumDistribution<T, Range::LMinValue, Range::LMaxValue>(spectra_data[i]));
    }
    return result;
}

} // namespace pbpt::radiometry
