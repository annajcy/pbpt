#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "pbpt/serde/scene_loader.hpp"
#include "pbpt/serde/scene_writer.hpp"

namespace {

struct CboxRenderTask {
    std::string scene_xml_rel;
    std::string output_stem;
    std::vector<int> spp_list;
    bool roundtrip_before_render{false};
};

struct RunOptions {
    std::filesystem::path output_root_rel{"output/cbox"};
    std::optional<int> spp_override{};
    std::optional<std::string> only_filter{};
    bool to_left_handed{true};
};

std::filesystem::path find_repo_root() {
    auto cur = std::filesystem::current_path();
    while (!cur.empty()) {
        if (std::filesystem::exists(cur / "asset/scene/cbox/cbox.xml")) {
            return cur;
        }
        if (cur == cur.root_path()) {
            break;
        }
        cur = cur.parent_path();
    }
    throw std::runtime_error("failed to locate repo root from current working directory");
}

void render_task(const std::filesystem::path& repo_root, const std::filesystem::path& output_root,
                 const CboxRenderTask& task, const std::optional<int>& spp_override, bool to_left_handed) {
    const auto scene_path = repo_root / task.scene_xml_rel;
    auto result = pbpt::serde::load_scene<double>(scene_path.string(), {.to_left_handed = false});

    const auto export_dir = output_root / "scene_export";
    std::filesystem::create_directories(export_dir);
    const auto exported_scene = export_dir / (task.output_stem + ".xml");
    pbpt::serde::write_scene(result, exported_scene.string(), {.to_left_handed = to_left_handed});

    if (task.roundtrip_before_render && !to_left_handed) {
        result = pbpt::serde::load_scene<double>(exported_scene.string(), {.to_left_handed = false});
    }

    std::vector<int> spp_values = task.spp_list;
    if (spp_values.empty()) {
        spp_values.push_back(result.spp);
    }
    if (spp_override.has_value()) {
        spp_values.assign(1, *spp_override);
    }

    for (int spp : spp_values) {
        std::string stem = task.output_stem;
        if (task.spp_list.size() > 1) {
            stem += "_" + std::to_string(spp);
        }
        const auto out_exr = output_root / (stem + ".exr");

        std::visit([&](auto& integrator) { integrator.render(result.scene, out_exr.string(), false, spp); },
                   result.integrator);
    }
}

std::vector<CboxRenderTask> build_tasks() {
    std::vector<int> default_spp_list{64};
    return {
        {"asset/scene/cbox/cbox.xml", "cbox", default_spp_list, true},
        {"asset/scene/cbox/cbox_cu_spec.xml", "cbox_cu_spec", default_spp_list, true},
        {"asset/scene/cbox/cbox_diele_spec.xml", "cbox_diele_spec", default_spp_list, true},
        {"asset/scene/cbox/cbox_checkerboard_texture.xml", "cbox_checkerboard_texture", default_spp_list, true},
        {"asset/scene/cbox/cbox_microfacet_cu_aniso.xml", "cbox_microfacet_cu_aniso", default_spp_list, true},
        {"asset/scene/cbox/cbox_microfacet_cu_iso.xml", "cbox_microfacet_cu_iso", default_spp_list, true},
        {"asset/scene/cbox/cbox_microfacet_diele_aniso.xml", "cbox_microfacet_diele_aniso", default_spp_list, true},
        {"asset/scene/cbox/cbox_microfacet_diele_iso.xml", "cbox_microfacet_diele_iso", default_spp_list, true},
        {"asset/scene/cbox/cbox_caustics_uv_sphere.xml", "cbox_caustics_uv_sphere", default_spp_list, true},
    };
}

RunOptions parse_args(int argc, char** argv) {
    RunOptions options{};
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i] ? argv[i] : "";
        if (arg == "--output" && i + 1 < argc) {
            options.output_root_rel = argv[++i];
            continue;
        }
        if (arg == "--spp" && i + 1 < argc) {
            const int spp = std::stoi(argv[++i]);
            if (spp <= 0) {
                throw std::runtime_error("--spp must be > 0");
            }
            options.spp_override = spp;
            continue;
        }
        if (arg == "--only" && i + 1 < argc) {
            options.only_filter = std::string(argv[++i]);
            continue;
        }
        if (arg == "--to-left-handed") {
            options.to_left_handed = true;
            continue;
        }
        throw std::runtime_error("Usage: cbox [--output <path>] [--spp <int>] [--only <keyword>] [--to-left-handed]");
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto repo_root = find_repo_root();
        const auto options = parse_args(argc, argv);

        std::filesystem::path output_root = options.output_root_rel;
        if (output_root.is_relative()) {
            output_root = repo_root / output_root;
        }
        std::filesystem::create_directories(output_root);

        const auto tasks = build_tasks();
        int failed = 0;
        int rendered = 0;
        for (const auto& task : tasks) {
            if (options.only_filter.has_value()) {
                const auto& token = *options.only_filter;
                if (task.scene_xml_rel.find(token) == std::string::npos &&
                    task.output_stem.find(token) == std::string::npos) {
                    continue;
                }
            }
            try {
                std::cout << "[cbox] render " << task.scene_xml_rel << std::endl;
                render_task(repo_root, output_root, task, options.spp_override, options.to_left_handed);
                ++rendered;
            } catch (const std::exception& e) {
                ++failed;
                std::cerr << "[cbox] failed " << task.scene_xml_rel << ": " << e.what() << std::endl;
            }
        }

        if (failed > 0) {
            std::cerr << "[cbox] done with failures: " << failed << std::endl;
            return 1;
        }
        std::cout << "[cbox] done, rendered tasks: " << rendered << ", outputs at: " << output_root << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[cbox] failed: " << e.what() << std::endl;
        return 1;
    }
}
