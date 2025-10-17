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

template<typename T>
class CsvSpectrumDataSource {
private:
    std::string file_path_;
    char delimiter_;
    
public:
    explicit CsvSpectrumDataSource(const std::string& path, char delimiter = ',') 
        : file_path_(path), delimiter_(delimiter) {}
    
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

// 通用的光谱创建函数模板
template<typename T, int N, typename Range>
inline auto make_spectra_from_csv(const std::string& csv_path) {
    static_assert(N > 0, "Number of spectra must be positive");

    auto source = CsvSpectrumDataSource<T>(csv_path);
    auto data = source.load_data();

    std::vector<std::vector<T>> spectra_data(N, std::vector<T>(Range::Count));

    for (int i = 0; i < Range::Count; ++i) {
        auto &[wavelength, values] = data[i];
        for (int j = 0; j < N && j < static_cast<int>(values.size()); ++j) {
            spectra_data[j][i] = values[j];
        }
    }

    std::array<radiometry::TabularSpectrumDistribution<T, Range::LMinValue, Range::LMaxValue>, N> result{};
    for (int i = 0; i < N; ++i) {
        result[i] = radiometry::TabularSpectrumDistribution<T, Range::LMinValue, Range::LMaxValue>(spectra_data[i]);
    }
    return result;
}

} // namespace pbpt::utils