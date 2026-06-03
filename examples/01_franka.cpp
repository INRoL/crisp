#include <crisp/crisp.hpp>

int main() {
  crisp::register_package_root("franka_description",
                               "assets/franka_description");

  auto builder = crisp::make_model_builder();

  builder->addRobot(
      {.path = "assets/franka_description/urdfs/fr3_franka_hand.urdf"});

  {
    auto &body = builder->addBody({.pos = {0.55, 0.0, 0.04},
                                   .mass = 0.2,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}},
                                  false);
    auto &geom = body.addGeom({.type = crisp::geometry_e::box,
                               .size = crisp::make_box_size(0.06, 0.06, 0.06),
                               .mu = 0.6});
    geom.createMaterial({.rgba = crisp::oklch(0.72f, 0.16f, 35.0f),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.12f, 0.12f, 0.12f},
                         .shininess = 24.0f});
  }

  builder->vis().cam.target = {0.3f, 0.0f, 0.25f};
  builder->vis().cam.distance = 1.6f;
  builder->vis().cam.azimuth = 140.0f;
  builder->vis().cam.elevation = -15.0f;
  builder->vis().cam.fovy = 45.0f;
  builder->addLight({.type = crisp::light_e::directional,
                     .enabled = true,
                     .cast_shadow = true,
                     .pos = {1.0f, -2.0f, 3.0f},
                     .dir = {-1.0f, 2.0f, -3.0f},
                     .ambient = {0.25f, 0.25f, 0.25f},
                     .diffuse = {0.8f, 0.8f, 0.8f},
                     .specular = {0.6f, 0.6f, 0.6f},
                     .attenuation = {1.0f, 0.0f, 0.0f},
                     .cutoff = 0.0f,
                     .exponent = 0.0f});

  for (auto joint :
       {"fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5",
        "fr3_joint6", "fr3_joint7", "fr3_finger_joint1", "fr3_finger_joint2"}) {
    builder->addActuator(joint, {.gain = {0, 1000, 100}});
  }

  auto m = builder->build();
  m->state.q0() << 0.0, 0.45, 0.0, -2.2, 0.0, 2.7, 0.75, 0.04, 0.04, 0.55, 0.0,
      0.04, 0.0, 0.0, 0.0, 1.0;
  m->act.u0() << 0.0, 0.45, 0.0, -2.2, 0.0, 2.7, 0.75, 0.04, 0.04;

  auto app = crisp::make_app(std::move(m));

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
