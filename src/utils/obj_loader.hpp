#pragma once
#include <array>
#include <cmath>
#include <functional>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "pbpt/math/normal.hpp"
#include "pbpt/math/point.hpp"

namespace pbpt::utils {

template<typename T>
struct ObjMeshData {
    std::vector<int> indices;
    std::vector<math::Point<T, 3>> positions;
    std::vector<math::Normal<T, 3>> normals;
    std::vector<math::Point<T, 2>> uvs;
};

struct ObjVertexKey {
    int v{-1};
    int vt{-1};
    int vn{-1};

    bool operator==(const ObjVertexKey& other) const {
        return v == other.v && vt == other.vt && vn == other.vn;
    }
};

struct ObjVertexKeyHash {
    std::size_t operator()(const ObjVertexKey& key) const {
        std::size_t h = std::hash<int>{}(key.v);
        h = h * 31u + std::hash<int>{}(key.vt);
        h = h * 31u + std::hash<int>{}(key.vn);
        return h;
    }
};

struct ObjVertexIndex {
    int v{0};
    int vt{0};
    int vn{0};
};

inline ObjVertexIndex parse_obj_vertex_token(const std::string& token) {
    ObjVertexIndex idx;
    auto first = token.find('/');
    if (first == std::string::npos) {
        idx.v = std::stoi(token);
        return idx;
    }

    idx.v = std::stoi(token.substr(0, first));
    auto second = token.find('/', first + 1);
    if (second == std::string::npos) {
        if (first + 1 < token.size()) {
            idx.vt = std::stoi(token.substr(first + 1));
        }
        return idx;
    }

    if (second > first + 1) {
        idx.vt = std::stoi(token.substr(first + 1, second - first - 1));
    }
    if (second + 1 < token.size()) {
        idx.vn = std::stoi(token.substr(second + 1));
    }
    return idx;
}

inline int to_zero_based_index(
    int idx,
    std::size_t count,
    const std::string& obj_path,
    const char* label
) {
    if (idx == 0) {
        throw std::runtime_error("OBJ missing index in " + obj_path);
    }
    int result = idx > 0 ? idx - 1 : static_cast<int>(count) + idx;
    if (result < 0 || result >= static_cast<int>(count)) {
        throw std::runtime_error("OBJ index out of range for " + std::string(label) + " in " + obj_path);
    }
    return result;
}

template<typename T>
ObjMeshData<T> load_obj_mesh(const std::string& obj_path) {
    std::ifstream fin(obj_path);
    if (!fin) {
        throw std::runtime_error("Cannot open OBJ file: " + obj_path);
    }

    std::vector<math::Point<T, 3>> obj_positions;
    std::vector<math::Normal<T, 3>> obj_normals;
    std::vector<math::Point<T, 2>> obj_uvs;

    // pass 1: load positions, normals, uvs
    std::string line;
    while (std::getline(fin, line)) {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line.erase(comment_pos);
        }
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag == "v") {
            T x{}, y{}, z{};
            if (iss >> x >> y >> z) {
                obj_positions.emplace_back(x, y, z);
            }
        } else if (tag == "vn") {
            T x{}, y{}, z{};
            if (iss >> x >> y >> z) {
                obj_normals.emplace_back(x, y, z);
            }
        } else if (tag == "vt") {
            T u{}, v{};
            if (iss >> u >> v) {
                obj_uvs.emplace_back(u, v);
            }
        }
    }

    fin.clear();
    fin.seekg(0);

    ObjMeshData<T> data;
    std::unordered_map<ObjVertexKey, int, ObjVertexKeyHash> remap;
    remap.reserve(obj_positions.size());

    bool all_have_normals = true;
    bool all_have_uvs = true;

    // pass 2: load faces

    while (std::getline(fin, line)) {
        auto comment_pos = line.find('#');
        if (comment_pos != std::string::npos) {
            line.erase(comment_pos);
        }
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        if (tag != "f") {
            continue;
        }

        std::vector<int> face_indices;
        std::string token;
        while (iss >> token) {
            ObjVertexIndex raw = parse_obj_vertex_token(token);
            int v = to_zero_based_index(raw.v, obj_positions.size(), obj_path, "v");
            int vt = raw.vt != 0 ? to_zero_based_index(raw.vt, obj_uvs.size(), obj_path, "vt") : -1;
            int vn = raw.vn != 0 ? to_zero_based_index(raw.vn, obj_normals.size(), obj_path, "vn") : -1;

            if (vt < 0) {
                all_have_uvs = false;
            }
            if (vn < 0) {
                all_have_normals = false;
            }

            ObjVertexKey key{v, vt, vn};
            auto it = remap.find(key);
            if (it == remap.end()) {
                int new_index = static_cast<int>(data.positions.size());
                data.positions.push_back(obj_positions[v]);
                if (vn >= 0) {
                    data.normals.push_back(obj_normals[vn]);
                } else {
                    data.normals.emplace_back(T(0), T(0), T(0));
                }
                if (vt >= 0) {
                    data.uvs.push_back(obj_uvs[vt]);
                } else {
                    data.uvs.emplace_back(T(0), T(0));
                }
                it = remap.emplace(key, new_index).first;
            }
            face_indices.push_back(it->second);
        }

        if (face_indices.size() < 3) {
            continue;
        }
        for (std::size_t i = 1; i + 1 < face_indices.size(); ++i) {
            data.indices.push_back(face_indices[0]);
            data.indices.push_back(face_indices[i]);
            data.indices.push_back(face_indices[i + 1]);
        }
    }

    if (!all_have_normals) {
        data.normals.clear();
    }
    if (!all_have_uvs) {
        data.uvs.clear();
    }
    return data;
}
}