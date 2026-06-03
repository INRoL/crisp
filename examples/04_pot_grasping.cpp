#include <crisp/crisp.hpp>

void my_control(Eigen::Ref<Eigen::VectorXd> act_in, double time,
                const Eigen::VectorXd &q0, const Eigen::VectorXd &q1,
                const Eigen::VectorXd &q2, const Eigen::VectorXd &q3) {
  double t1 = 1.0;
  double t2 = 8.0;
  double t3 = 12.0;
  double t4 = 16.0;
  double t5 = 20.0;

  if (time <= t1) {
    act_in.head(7) = q0;
    act_in.tail(16).setZero();
  } else if (time >= t1 && time <= t2) {
    act_in(8) = 0.2 * (time - t1);
    act_in(9) = 0.2 * (time - t1);
    act_in(10) = 0.2 * (time - t1);
    act_in(12) = 0.2 * (time - t1);
    act_in(13) = 0.2 * (time - t1);
    act_in(14) = 0.2 * (time - t1);
    act_in(16) = 0.2 * (time - t1);
    act_in(17) = 0.2 * (time - t1);
    act_in(18) = 0.2 * (time - t1);
  } else if (time <= t3) {
    act_in.head(7) = ((t3 - time) * q0 + (time - t2) * q1) / 4.0;
  } else if (time <= t4) {
    act_in.head(7) = ((t4 - time) * q1 + (time - t3) * q2) / 4.0;
  } else if (time <= t5) {
    act_in.head(7) = ((t5 - time) * q2 + (time - t4) * q3) / 4.0;
    act_in(8) = 0.2 * (t5 - time) + 0.6;
    act_in(9) = 0.2 * (t5 - time) + 0.6;
    act_in(10) = 0.2 * (t5 - time) + 0.6;
    act_in(12) = 0.2 * (t5 - time) + 0.6;
    act_in(13) = 0.2 * (t5 - time) + 0.6;
    act_in(14) = 0.2 * (t5 - time) + 0.6;
    act_in(16) = 0.2 * (t5 - time) + 0.6;
    act_in(17) = 0.2 * (t5 - time) + 0.6;
    act_in(18) = 0.2 * (t5 - time) + 0.6;
  } else {
    act_in.head(7) = q3;
  }
}

int main() {
  auto builder = crisp::make_model_builder();

  // robot
  builder->addRobot({.path = "assets/panda/pallegro.urdf"});

  { // box
    auto &body = builder->addBody({.pos = {0.5, -0.1, 0.175},
                                   .mass = 0.1,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}},
                                  true);
    auto &geom = body.addGeom({.type = crisp::geometry_e::box,
                               .size = crisp::make_box_size(0.4, 0.4, 0.4)});
    geom.createMaterial({.rgba = crisp::oklch(0.78f, 0.04f, 250.0f),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.1f, 0.1f, 0.1f},
                         .shininess = 32.0f});
  }

  // pot
  builder->addRobot({.path = "assets/dish/pot.urdf"}, false);

  for (auto joint :
       {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"}) {
    builder->addActuator(joint, {.gain = {0, 2000, 200}});
  }
  for (auto joint :
       {"allegro_joint0", "allegro_joint1", "allegro_joint2", "allegro_joint3",
        "allegro_joint4", "allegro_joint5", "allegro_joint6", "allegro_joint7",
        "allegro_joint8", "allegro_joint9", "allegro_joint10",
        "allegro_joint11", "allegro_joint12", "allegro_joint13",
        "allegro_joint14", "allegro_joint15"}) {
    builder->addActuator(joint, {.gain = {0, 2000, 200}});
  }

  builder->vis().cam.target = {0.45f, -0.08f, 0.4f};
  builder->vis().cam.distance = 1.35f;
  builder->vis().cam.azimuth = 145.0f;
  builder->vis().cam.elevation = -24.0f;
  builder->vis().cam.fovy = 42.0f;
  builder->addLight({.type = crisp::light_e::directional,
                     .enabled = true,
                     .cast_shadow = true,
                     .pos = {1.5f, -2.0f, 3.0f},
                     .dir = {-1.5f, 2.0f, -3.0f},
                     .ambient = {0.22f, 0.22f, 0.22f},
                     .diffuse = {0.75f, 0.75f, 0.75f},
                     .specular = {0.4f, 0.4f, 0.4f},
                     .attenuation = {1.0f, 0.0f, 0.0f},
                     .cutoff = 0.0f,
                     .exponent = 0.0f});
  builder->opt().sol.type = crisp::solver_e::sub_admm;

  auto m = builder->build();

  m->state.q0().segment(0, 23) << -0.11535, 0.70988, 0.66745, -0.99929,
      -0.02496, 0.89239, -2.57345, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0;
  m->state.q0().segment(44 - 21, 7) << 0.45, -0.0, 0.38, 0, 0, 0, 1;

  auto app = crisp::make_app(std::move(m));

  Eigen::VectorXd q0(7), q1(7), q2(7), q3(7);
  q0 << -0.11535, 0.70988, 0.66745, -0.99929, -0.02496, 0.89239, -2.57345;
  q1 << -0.23195, 0.33637, 0.69232, -1.23754, 0.19635, 0.80451, -2.54567;
  q2 << -0.93472, 0.31925, 1.14121, -1.36652, -0.11363, 0.73554, -2.89141;
  q3 << -1.08551, 0.32097, 1.25832, -1.59726, -0.13407, 0.92468, -2.89724;
  app->setControl([&](auto const &m, auto const &d, auto u, auto udot) {
    my_control(u, d.sim.time, q0, q1, q2, q3);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
