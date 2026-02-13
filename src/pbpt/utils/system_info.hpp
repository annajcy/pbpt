#pragma once

#include <array>
#include <chrono>
#include <ctime>
#include <format>
#include <string>
#include <string_view>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <fstream>

#if defined(_WIN32)
#   ifndef NOMINMAX
#       define NOMINMAX
#   endif
#   include <windows.h>
#   include <intrin.h>
#else
#   include <sys/utsname.h>
#   if defined(__APPLE__)
#       include <sys/sysctl.h>
#   elif defined(__linux__)
#       include <sys/sysinfo.h>
#   endif
#endif

namespace pbpt::utils {

struct DateTime {
    int year{};
    int month{};
    int day{};
    int hour{};
    int minute{};
    int second{};
};

inline DateTime current_datetime() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t time = std::chrono::system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &time);
#else
    localtime_r(&time, &tm);
#endif

    DateTime dt{};
    dt.year = tm.tm_year + 1900;
    dt.month = tm.tm_mon + 1;
    dt.day = tm.tm_mday;
    dt.hour = tm.tm_hour;
    dt.minute = tm.tm_min;
    dt.second = tm.tm_sec;
    return dt;
}

inline std::string to_string(const DateTime &dt) {
    return std::format("{:04}-{:02}-{:02}_{:02}-{:02}-{:02}",
        dt.year, dt.month, dt.day,
        dt.hour, dt.minute, dt.second);
}

inline void trim_in_place(std::string &value) {
    const auto start = value.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
        value.clear();
        return;
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    value = value.substr(start, end - start + 1);
}

inline std::string read_value_from_file(const char *path, std::string_view prefix) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return {};
    }
    std::string line;
    while (std::getline(file, line)) {
        if (line.starts_with(prefix)) {
            const auto colon = line.find(':');
            if (colon != std::string::npos && colon + 1 < line.size()) {
                auto value = line.substr(colon + 1);
                trim_in_place(value);
                return value;
            }
        }
    }
    return {};
}

inline std::string read_first_line(const char *path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        return {};
    }
    std::string line;
    if (std::getline(file, line)) {
        trim_in_place(line);
        return line;
    }
    return {};
}

inline std::string format_bytes(std::uint64_t bytes) {
    constexpr std::array<const char *, 6> suffixes{{"B", "KB", "MB", "GB", "TB", "PB"}};
    double value = static_cast<double>(bytes);
    std::size_t idx = 0;
    while (value >= 1024.0 && idx + 1 < suffixes.size()) {
        value /= 1024.0;
        ++idx;
    }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), value >= 10.0 ? "%.0f%s" : "%.1f%s", value, suffixes[idx]);
    return std::string(buffer);
}

inline std::string os_string() {
#if defined(_WIN32)
    OSVERSIONINFOEXA info{};
    info.dwOSVersionInfoSize = sizeof(info);
    if (GetVersionExA(reinterpret_cast<OSVERSIONINFOA *>(&info))) {
        char buffer[64];
        std::snprintf(
            buffer,
            sizeof(buffer),
            "Windows %lu.%lu (build %lu)",
            static_cast<unsigned long>(info.dwMajorVersion),
            static_cast<unsigned long>(info.dwMinorVersion),
            static_cast<unsigned long>(info.dwBuildNumber)
        );
        return std::string(buffer);
    }
    return "Windows";
#else
    struct utsname uts{};
    if (uname(&uts) == 0) {
        std::string os = uts.sysname;
        if (std::strlen(uts.release) != 0) {
            os.push_back(' ');
            os.append(uts.release);
        }
        return os;
    }
#   if defined(__APPLE__)
    return "macOS";
#   elif defined(__linux__)
    return "Linux";
#   else
    return "Unknown OS";
#   endif
#endif
}

inline std::string architecture_string() {
#if defined(__aarch64__) || defined(_M_ARM64)
    return "ARM64";
#elif defined(__arm__) || defined(_M_ARM)
    return "ARM";
#elif defined(__x86_64__) || defined(_M_X64)
    return "x86_64";
#elif defined(__i386__) || defined(_M_IX86)
    return "x86";
#elif defined(__powerpc64__)
    return "PowerPC64";
#else
    return "Unknown";
#endif
}

inline std::string cpu_brand_string() {
#if defined(_WIN32)
    int cpu_info[4] = {0};
    __cpuid(cpu_info, 0x80000000);
    const unsigned max_id = static_cast<unsigned>(cpu_info[0]);
    char brand[49] = {};
    if (max_id >= 0x80000004) {
        __cpuid(cpu_info, 0x80000002);
        std::memcpy(brand, cpu_info, sizeof(cpu_info));
        __cpuid(cpu_info, 0x80000003);
        std::memcpy(brand + 16, cpu_info, sizeof(cpu_info));
        __cpuid(cpu_info, 0x80000004);
        std::memcpy(brand + 32, cpu_info, sizeof(cpu_info));
        brand[48] = '\0';
        std::string result(brand);
        trim_in_place(result);
        return result;
    }
    return "CPU";
#elif defined(__APPLE__)
    char buffer[256] = {};
    size_t size = sizeof(buffer);
    if (sysctlbyname("machdep.cpu.brand_string", buffer, &size, nullptr, 0) == 0 && size > 0) {
        std::string brand(buffer, size - 1);
        trim_in_place(brand);
        return brand;
    }
    return "CPU";
#elif defined(__linux__)
    auto model = read_value_from_file("/proc/cpuinfo", "model name");
    if (!model.empty()) {
        return model;
    }
    return "CPU";
#else
    return "CPU";
#endif
}

inline std::uint64_t total_memory_bytes() {
#if defined(_WIN32)
    MEMORYSTATUSEX status{};
    status.dwLength = sizeof(status);
    if (GlobalMemoryStatusEx(&status)) {
        return status.ullTotalPhys;
    }
    return 0;
#elif defined(__APPLE__)
    std::uint64_t mem = 0;
    size_t size = sizeof(mem);
    if (sysctlbyname("hw.memsize", &mem, &size, nullptr, 0) == 0) {
        return mem;
    }
    return 0;
#elif defined(__linux__)
    struct sysinfo info {};
    if (sysinfo(&info) == 0) {
        return static_cast<std::uint64_t>(info.totalram) * static_cast<std::uint64_t>(info.mem_unit);
    }
    return 0;
#else
    return 0;
#endif
}

inline std::string gpu_string() {
#if defined(_WIN32)
    DISPLAY_DEVICEA device{};
    device.cb = sizeof(device);
    for (DWORD i = 0; EnumDisplayDevicesA(nullptr, i, &device, 0); ++i) {
        if (device.StateFlags & DISPLAY_DEVICE_PRIMARY_DEVICE) {
            return std::string(device.DeviceString);
        }
    }
    return "Unknown GPU";
#elif defined(__APPLE__)
    return "Apple GPU";
#elif defined(__linux__)
    if (auto model = read_value_from_file("/proc/driver/nvidia/gpus/0/information", "Model"); !model.empty()) {
        return model;
    }
    if (auto fb = read_first_line("/sys/class/graphics/fb0/name"); !fb.empty()) {
        return fb;
    }
    if (auto driver = read_value_from_file("/sys/class/drm/card0/device/uevent", "DRIVER"); !driver.empty()) {
        return driver;
    }
    return "Unknown GPU";
#else
    return "Unknown GPU";
#endif
}

struct SystemInfo {
    DateTime datetime;
    std::string os;
    std::string architecture;
    std::string cpu;
    std::string gpu;
    std::uint32_t cores{};
    std::uint64_t ram_bytes{};
};

inline SystemInfo system_info() {
    SystemInfo info{};
    info.datetime = current_datetime();
    info.os = os_string();
    info.architecture = architecture_string();
    info.cpu = cpu_brand_string();
    info.gpu = gpu_string();
    info.cores = std::max(1u, std::thread::hardware_concurrency());
    info.ram_bytes = total_memory_bytes();
    return info;
}

inline std::string to_string(const SystemInfo &info) {
    std::string result;
    result.reserve(128);
    result.append("OS: ").append(info.os);
    result.append(" (").append(info.architecture).append(")\n");
    result.append("CPU: ").append(info.cpu);
    result.append(" (").append(std::to_string(info.cores)).append(" cores)\n");
    result.append("RAM: ").append(format_bytes(info.ram_bytes)).append("\n");
    result.append("GPU: ").append(info.gpu);
    return result;
}

}
