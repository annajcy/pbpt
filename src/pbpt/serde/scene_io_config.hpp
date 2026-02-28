#pragma once

namespace pbpt::serde {

struct SceneLoadConfig {
    bool load_light_sampler{false};
    bool load_integrator_rr{false};
    bool load_microfacet_dist{false};
    bool to_left_handed{true};
};

struct SceneWriteConfig {
    bool write_light_sampler{false};
    bool write_integrator_rr{false};
    bool write_microfacet_dist{false};
    bool to_left_handed{true};
};

}  // namespace pbpt::serde
