#include <crisp/crisp.hpp>

struct nn_sdf_t {
  double phi;
  Eigen::Vector3d grad;
};

nn_sdf_t cylinder_sdf(Eigen::Vector3d p, const double &r, const double &h);

int peg_eval(crisp::Ref<crisp::Vector3r const> x_rel,
             crisp::Ref<crisp::VectorXr const> param, int, crisp::real_t &phi,
             crisp::Ref<crisp::Vector3r> grad, crisp::Ref<crisp::Matrix3r>) {
  auto out = cylinder_sdf(x_rel, param[0], param[1]);
  phi = out.phi;
  grad = out.grad;
  return crisp::sdf_eval_grad;
}

void peg_aabb(crisp::Ref<crisp::Matrix3r const>,
              crisp::Ref<crisp::VectorXr const> param,
              crisp::Ref<crisp::Vector6r> aabb) {
  double r = param[0];
  double h = param[1];
  aabb << -1.5 * r, -1.5 * r, -h, 1.5 * r, 1.5 * r, h;
}

nn_sdf_t cylinder_sdf(Eigen::Vector3d p, const double &r, const double &h) {
  Eigen::Vector2d p_2d(std::sqrt(p(0) * p(0) + p(1) * p(1)), p(2));
  double xy = std::sqrt(p(0) * p(0) + p(1) * p(1));
  double radial_dist = xy - r;
  double axial_dist = abs(p(2)) - 0.5 * h;

  double radial_out = std::max(radial_dist, 0.0);
  double axial_out = std::max(axial_dist, 0.0);

  double dist;
  Eigen::Vector3d grad;

  if (radial_out > 0.0 || axial_out > 0.0) {
    dist = sqrt(radial_out * radial_out + axial_out * axial_out);

    Eigen::Vector3d g(0.0, 0.0, 0.0);
    if (radial_out > 1e-12 && xy > 1e-12) {
      g(0) = (radial_out / dist) / xy * p(0);
      g(1) = (radial_out / dist) / xy * p(1);
    }
    if (axial_out > 1e-12) {
      g(2) = (axial_out / dist) * (p(2) >= 0.0 ? 1.0 : -1.0);
    }
    grad = g.normalized();
  } else {
    dist = std::max(radial_dist, axial_dist);

    if (axial_dist <= radial_dist) {
      if (xy > 1e-12) {
        grad << p(0) / xy, p(1) / xy, 0.0;
      } else {
        grad << 1.0, 0.0, 0.0;
      }
    } else {
      grad << 0.0, 0.0, (p(2) >= 0.0 ? 1.0 : -1.0);
    }
  }
  return nn_sdf_t{dist, grad};
}

void my_control(Eigen::Ref<Eigen::VectorXd> act_in, double time,
                const Eigen::VectorXd &q_0, const Eigen::VectorXd &q_des) {
  Eigen::VectorXd q_i;
  Eigen::VectorXd q_f;
  if (time < 1.0) {
    q_i = q_0;
    q_f = q_des.segment(13 * 0, 13);
    act_in = q_i + (q_f - q_i) * (time - 0.0) / 1.0;
  } else if (time < 2.0) {
    q_i = q_des.segment(13 * 0, 13);
    q_f = q_des.segment(13 * 1, 13);
    act_in = q_i + (q_f - q_i) * (time - 1.0) / 1.0;
  } else if (time < 3.0) {
    q_i = q_des.segment(13 * 1, 13);
    q_f = q_des.segment(13 * 2, 13);
    act_in = q_i + (q_f - q_i) * (time - 2.0) / 1.0;
  } else if (time < 3.5) {
    act_in = q_des.segment(13 * 2, 13);
  } else if (time < 5.0) {
    q_i = q_des.segment(13 * 2, 13);
    q_f = q_des.segment(13 * 3, 13);
    act_in = q_i + (q_f - q_i) * (time - 3.5) / 1.5;
  } else if (time < 6.0) {
    q_i = q_des.segment(13 * 3, 13);
    q_f = q_des.segment(13 * 4, 13);
    act_in = q_i + (q_f - q_i) * (time - 5.0) / 1.0;
  } else if (time < 7.0) {
    q_i = q_des.segment(13 * 4, 13);
    q_f = q_des.segment(13 * 5, 13);
    act_in = q_i + (q_f - q_i) * (time - 6.0) / 1.0;
  } else if (time < 9.0) {
    q_i = q_des.segment(13 * 5, 13);
    q_f = q_des.segment(13 * 6, 13);
    act_in = q_i + (q_f - q_i) * (time - 7.0) / 2.0;
  } else {
    q_f = q_des.segment(13 * 6, 13);
    act_in = q_f;
  }
}

int main() {
  int sdf_type_0 = crisp::register_sdf(peg_eval, peg_aabb, 2);

  auto builder = crisp::make_model_builder();

  // robot
  builder->addRobot({.path = "assets/panda/franka_robotiq2f85.urdf"});

  { // box
    auto &body = builder->addBody({.pos = {0.5, 0.0, 0.15},
                                   .mass = 1.0,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}},
                                  true);
    auto &geom = body.addGeom({.type = crisp::geometry_e::box,
                               .size = crisp::make_box_size(0.6, 1.0, 0.3)});
    geom.createMaterial({.rgba = crisp::oklch(0.78f, 0.04f, 250.0f),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.1f, 0.1f, 0.1f},
                         .shininess = 32.0f});
  }

  { // peg
    auto &body =
        builder->addBody({.pos = {0.5, -0.1, 0.3 + 0.025 + 0.003 + 1e-6},
                          .mass = 0.079,
                          .inertia = {0.001, 0.001, 0.001, 0, 0, 0}},
                         false);
    auto &geom_r = body.addGeom(
        {.visual = true, .con_type = 0, .con_affinity = 0, .mu = 0.15});
    geom_r.createMesh({.path = "assets/industrealkit/peg_round_16mm.obj",
                       .scale = {1, 1, 1}});
    geom_r.createMaterial({.rgba = crisp::oklch(0.8f, 0.0f, 0)});
    auto &geom_c = body.addGeom({.visual = false});
    geom_c.createSDF(
        {.type = sdf_type_0,
         .param = crisp::to_view(Eigen::VectorXd{{0.016 / 2.0, 0.05}})});
  }

  { // peg tray
    auto &body = builder->addBody({.pos = {0.5, -0.1, 0.303},
                                   .mass = 0.02,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}},
                                  true);
    auto &geom = body.addGeom({.mu = 0.15});
    geom.createMesh({.path = "assets/industrealkit/peg_tray_round_16mm.obj",
                     .scale = {1, 1, 1}});
    geom.createMaterial({.rgba = crisp::oklch(0.8f, 0.2f, 140.0f),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.1f, 0.1f, 0.1f},
                         .shininess = 32.0f});
  }

  { // hole
    auto &body = builder->addBody({.pos = {0.5, 0.0, 0.303},
                                   .mass = 0.02,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}},
                                  true);
    auto &geom = body.addGeom({.mu = 0.15});
    geom.createMesh({.path = "assets/industrealkit/hole_round_16mm.obj",
                     .scale = {1, 1, 1}});
    geom.createMaterial({.rgba = crisp::oklch(0.8f, 0.2f, 140.0f),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.1f, 0.1f, 0.1f},
                         .shininess = 32.0f});
  }

  for (auto joint :
       {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"}) {
    builder->addActuator(joint, {.gain = {0, 2000, 200}});
  }
  for (auto joint : {"finger_joint", "left_inner_knuckle_joint",
                     "left_inner_finger_joint", "right_outer_knuckle_joint",
                     "right_inner_knuckle_joint", "right_inner_finger_joint"}) {
    builder->addActuator(joint, {.gain = {0, 200, 20}});
  }

  builder->vis().cam.target = {0.5f, -0.05f, 0.32f};
  builder->vis().cam.distance = 1.0f;
  builder->vis().cam.azimuth = 180.0f;
  builder->vis().cam.elevation = -20.0f;
  builder->vis().cam.fovy = 36.0f;
  builder->addLight({.type = crisp::light_e::directional,
                     .enabled = true,
                     .cast_shadow = true,
                     .pos = {1.2f, -1.6f, 2.6f},
                     .dir = {-1.2f, 1.6f, -2.6f},
                     .ambient = {0.2f, 0.2f, 0.2f},
                     .diffuse = {0.8f, 0.8f, 0.8f},
                     .specular = {0.45f, 0.45f, 0.45f},
                     .attenuation = {1.0f, 0.0f, 0.0f},
                     .cutoff = 0.0f,
                     .exponent = 0.0f});
  builder->opt().con.erp = 0.01;
  builder->opt().sol.type = crisp::solver_e::sub_admm;
  builder->cap().ncon_max = 200;

  auto m = builder->build();

  m->state.q0() << 0.887293, -0.305106, -0.941466, -1.88319, -0.247274, 1.69601,
      0, 0, 0, 0, 0, 0, 0, 0.5, -0.1, 0.3 + 0.025 + 0.003 + 1e-6, 0, 0, 0, 1;

  auto app = crisp::make_app(std::move(m));

  Eigen::VectorXd q0_robot(13), q_des(13 * 7);

  q0_robot << 0.887293, -0.305106, -0.941466, -1.88319, -0.247274, 1.69601, 0,
      0, 0, 0, 0, 0, 0;

  double theta = 0.65;

  q_des << 0.887293, -0.305106, -0.941466, -1.88319, -0.247274, 1.69601, 0, 0,
      0, 0, 0, 0, 0, // stay
      0.864693, -0.297066, -0.956013, -2.03104, -0.251228, 1.84777, 0, 0, 0, 0,
      0, 0, 0, // down
      0.864693, -0.297066, -0.956013, -2.03104, -0.251228, 1.84777, 0, theta,
      theta, -theta, -theta, -theta, theta, // grip
      0.887839, -0.305098, -0.941652, -1.88168, -0.247257, 1.6946, 0, theta,
      theta, -theta, -theta, -theta, theta, // up
      4.44089e-16, -0.195957, -4.45208e-16, -1.93015, 4.0453e-17, 1.7342, 0,
      theta, theta, -theta, -theta, -theta, theta, // move
      4.44089e-16, -0.197213, -4.45819e-16, -1.99514, 3.31717e-17, 1.79793, 0,
      theta, theta, -theta, -theta, -theta, theta, // contact
      4.44089e-16, -0.177692, -4.5125e-16, -2.141, 1.3881e-17, 1.96331, 0,
      theta, theta, -theta, -theta, -theta, theta; // insert

  double alpha = 0.5;
  q_des.segment(13 * 6, 7) =
      alpha * q_des.segment(13 * 6, 7) + (1 - alpha) * q_des.segment(13 * 5, 7);

  app->setControl([&](auto const &m, auto const &d, auto u, auto udot) {
    my_control(u, d.sim.time, q0_robot, q_des);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
