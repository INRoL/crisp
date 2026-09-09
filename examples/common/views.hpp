#pragma once

#include <crisp/crisp.hpp>

namespace utils {

inline void set_assembly_view(crisp::ModelBuilder &builder) {
  auto &camera = builder.vis().cam;
  camera.target = {0.50f, -0.05f, 0.35f};
  camera.distance = 0.24f;
  camera.azimuth = -125.0f;
  camera.elevation = -15.0f;
  camera.fovy = 60.0f;
}

inline void add_assembly_lights(crisp::ModelBuilder &builder) {
  builder.addLight({.type = crisp::light_e::directional,
                    .enabled = true,
                    .cast_shadow = true,
                    .shadow_extent = 0.7f,
                    .pos = {0.85f, 0.35f, 1.1f},
                    .dir = {-0.35f, -0.40f, -0.75f},
                    .ambient = {0.34f, 0.34f, 0.34f},
                    .diffuse = {0.65f, 0.65f, 0.65f},
                    .specular = {0.85f, 0.85f, 0.85f},
                    .attenuation = {1.0f, 0.0f, 0.0f},
                    .cutoff = 30.0f,
                    .exponent = 0.0f});
  builder.addLight({.type = crisp::light_e::directional,
                    .enabled = true,
                    .cast_shadow = false,
                    .pos = {0.2f, -1.0f, 0.8f},
                    .dir = {-0.3f, 1.0f, -0.35f},
                    .ambient = {0.0f, 0.0f, 0.0f},
                    .diffuse = {0.20f, 0.20f, 0.20f},
                    .specular = {0.85f, 0.85f, 0.85f},
                    .attenuation = {1.0f, 0.0f, 0.0f},
                    .cutoff = 10.0f,
                    .exponent = 0.0f});
  builder.addLight({.type = crisp::light_e::directional,
                    .enabled = true,
                    .cast_shadow = false,
                    .pos = {-1.0f, 0.2f, 0.8f},
                    .dir = {1.0f, -0.2f, -0.5f},
                    .ambient = {0.0f, 0.0f, 0.0f},
                    .diffuse = {0.12f, 0.12f, 0.12f},
                    .specular = {1.0f, 1.0f, 1.0f},
                    .attenuation = {1.0f, 0.0f, 0.0f},
                    .cutoff = 10.0f,
                    .exponent = 0.0f});
}

} // namespace utils
