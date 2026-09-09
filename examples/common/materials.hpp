#pragma once

#include <crisp/crisp.hpp>

namespace utils {

inline void set_gripper_materials(crisp::model_t &model) {
  for (int g = 0; g < model.size.ngeom; ++g) {
    const int body = model.geom.bodyid[g];
    const int material = model.geom.matid[g];
    if (body < 0 || material < 0 || !model.geom.visual[g])
      continue;

    const int name_id = model.body.nameid[body];
    if (name_id < 0)
      continue;

    const auto name = model.str.names[name_id].view();
    const bool is_base = name == "robotiq_85_base_link";
    const bool is_pad =
        name == "left_inner_finger" || name == "right_inner_finger";
    const bool is_finger =
        name == "left_outer_knuckle" || name == "left_outer_finger" ||
        name == "left_inner_knuckle" || name == "right_outer_knuckle" ||
        name == "right_outer_finger" || name == "right_inner_knuckle";
    if (!is_base && !is_pad && !is_finger)
      continue;

    const float color = is_pad ? 0.08f : 0.1f;
    const float specular = is_base ? 0.08f : (is_pad ? 0.28f : 0.85f);
    model.mat.rgba[material] = {color, color, color, 1.0f};
    model.mat.emission[material].setZero();
    model.mat.specular[material] = {specular, specular, specular};
    model.mat.shininess[material] = is_base ? 6.0f : (is_pad ? 14.0f : 28.0f);
  }
}

} // namespace utils
