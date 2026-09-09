#include <crisp/crisp.hpp>

#include <algorithm>
#include <cmath>

#include "common/materials.hpp"
#include "common/views.hpp"

namespace {

constexpr Eigen::Index kControlSize = 13;
constexpr double kFilletRadius = 5e-4;
constexpr double kFiniteDifferenceStep = 1e-8;

double peg_distance(const Eigen::Vector3d &point, double radius,
                    double half_height, double fillet_radius) {
  const double radial_distance =
      std::sqrt(point.x() * point.x() + point.y() * point.y());
  const double radial_offset = radial_distance - radius + fillet_radius;
  const double axial_offset = std::abs(point.z()) - half_height;
  const double radial_outside = std::max(radial_offset, 0.0);
  const double axial_outside = std::max(axial_offset, 0.0);
  const double outside_distance = std::sqrt(radial_outside * radial_outside +
                                            axial_outside * axial_outside);
  const double inside_distance =
      std::min(std::max(radial_offset, axial_offset), 0.0);
  return inside_distance + outside_distance - fillet_radius;
}

Eigen::Vector3d peg_gradient(const Eigen::Vector3d &point, double radius,
                             double half_height, double fillet_radius) {
  const double distance =
      peg_distance(point, radius, half_height, fillet_radius);
  Eigen::Vector3d gradient;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d displaced = point;
    displaced[axis] += kFiniteDifferenceStep;
    gradient[axis] =
        (peg_distance(displaced, radius, half_height, fillet_radius) -
         distance) /
        kFiniteDifferenceStep;
  }
  return gradient;
}

int evaluate_peg_sdf(crisp::Ref<crisp::Vector3r const> x_rel,
                     crisp::Ref<crisp::VectorXr const> param, int requested,
                     crisp::real_t &phi, crisp::Ref<crisp::Vector3r> grad,
                     crisp::Ref<crisp::Matrix3r>) {
  const double radius = param[0];
  const double half_height = param[1] * 0.5 - kFilletRadius;
  const Eigen::Vector3d point = x_rel;

  phi = peg_distance(point, radius, half_height, kFilletRadius);
  if (!(requested & crisp::sdf_eval_grad))
    return 0;

  Eigen::Vector3d gradient =
      peg_gradient(point, radius, half_height, kFilletRadius);
  const double norm = gradient.norm();
  grad = norm > 1e-12 ? (gradient / norm).eval() : Eigen::Vector3d::Zero();
  return crisp::sdf_eval_grad;
}

void compute_peg_aabb(crisp::Ref<crisp::VectorXr const> param,
                      crisp::Ref<crisp::Vector6r> aabb) {
  const double radius = param[0];
  const double half_height = param[1] * 0.5;
  aabb << -radius, -radius, -half_height, radius, radius, half_height;
}

void update_control(Eigen::Ref<Eigen::VectorXd> act_in, double time,
                    const Eigen::VectorXd &q_0, const Eigen::VectorXd &q_des) {
  const auto target = [&](Eigen::Index index) {
    return q_des.segment(kControlSize * index, kControlSize);
  };
  const auto interpolate = [&](const auto &initial, const auto &final,
                               double start, double duration) {
    act_in = initial + (final - initial) * (time - start) / duration;
  };

  if (time < 1.0) {
    interpolate(q_0, target(0), 0.0, 1.0);
  } else if (time < 2.0) {
    interpolate(target(0), target(1), 1.0, 1.0);
  } else if (time < 3.0) {
    interpolate(target(1), target(2), 2.0, 1.0);
  } else if (time < 3.5) {
    act_in = target(2);
  } else if (time < 5.0) {
    interpolate(target(2), target(3), 3.5, 1.5);
  } else if (time < 6.0) {
    interpolate(target(3), target(4), 5.0, 1.0);
  } else if (time < 7.0) {
    interpolate(target(4), target(5), 6.0, 1.0);
  } else if (time < 9.0) {
    interpolate(target(5), target(6), 7.0, 2.0);
  } else {
    act_in = target(6);
  }
}

} // namespace

int main() {
  constexpr double steel_mu = 0.15;
  constexpr double pla_mu = 0.35;
  constexpr double pla_k = 1e5;

  const int peg_sdf_type = crisp::register_sdf({.name = "peg_insertion/peg",
                                                .eval = evaluate_peg_sdf,
                                                .aabb = compute_peg_aabb,
                                                .nparam = 2});

  auto builder = crisp::make_model_builder();

  builder->addRobot({.path = "assets/panda/franka_robotiq2f85.urdf"});

  {
    auto &body = builder->addBody({.pos = {0.5, 0.0, 0.15},
                                   .mass = 1.0,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}});
    auto &geom = body.addGeom({.type = crisp::geometry_e::box,
                               .size = crisp::make_box_size(0.6, 1.0, 0.3)});
    geom.createMaterial({.rgba = crisp::oklch(0.42f, 0.006f, 250.0f),
                         .specular = {0.07f, 0.07f, 0.07f},
                         .shininess = 16.0f});
  }

  {
    auto &body = builder->addBody(
        {.pos = {0.5, -0.1, 0.328},
         .mass = 0.07792,
         .inertia = {1.7426e-05, 1.7426e-05, 2.4623e-06, 0, 0, 0}},
        false);
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh({.path = "assets/industrealkit/peg_round_16mm.obj"});
    visual_geom.createMaterial({.rgba = {0.9843f, 0.9725f, 0.8784f, 1.0f},
                                .specular = {0.16f, 0.16f, 0.16f},
                                .shininess = 36.0f});
    auto &collision_geom = body.addGeom({.visual = false, .mu = steel_mu});
    collision_geom.createSDF(
        {.type = peg_sdf_type,
         .param = crisp::to_view(Eigen::VectorXd{{0.008, 0.05}})});
  }

  {
    auto &body = builder->addBody({.pos = {0.5, -0.1, 0.303},
                                   .mass = 0.02,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}});
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh(
        {.path = "assets/industrealkit/peg_tray_round_16mm.obj"});
    visual_geom.createMaterial({.rgba = {0.7416f, 0.9150f, 1.0000f, 1.0f},
                                .specular = {0.18f, 0.18f, 0.18f},
                                .shininess = 48.0f});
    auto &collision_geom =
        body.addGeom({.visual = false, .mu = pla_mu, .k = pla_k});
    collision_geom.createMesh(
        {.path = "assets/industrealkit/peg_tray_round_16mm_remesh1.5mm.obj"});
  }

  {
    auto &body = builder->addBody({.pos = {0.5, 0.0, 0.303},
                                   .mass = 0.02,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}});
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh(
        {.path = "assets/industrealkit/hole_round_16mm.obj"});
    visual_geom.createMaterial({.rgba = {0.3643f, 0.6225f, 0.8346f, 1.0f},
                                .specular = {0.09f, 0.09f, 0.09f},
                                .shininess = 20.0f});
    auto &collision_geom =
        body.addGeom({.visual = false, .mu = pla_mu, .k = pla_k});
    collision_geom.createMesh(
        {.path = "assets/industrealkit/hole_round_16mm_remesh1.5mm.obj"});
  }

  for (const auto joint :
       {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"}) {
    builder->addActuator(joint, {.gain = {0, 20000, 2000}});
  }
  for (const auto joint :
       {"finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint",
        "right_outer_knuckle_joint", "right_inner_knuckle_joint",
        "right_inner_finger_joint"}) {
    builder->addActuator(joint, {.gain = {0, 200, 20}});
  }

  utils::set_assembly_view(*builder);
  builder->vis().bg.setOnes();
  utils::add_assembly_lights(*builder);

  builder->opt().dt = 5e-3;
  builder->opt().con.margin = {0.001, 0.001, 0.002};
  builder->opt().con.erp = 0.1;
  builder->opt().sol.type = crisp::solver_e::canal;
  builder->opt().sol.max_iter = 20;
  builder->cap().ncon_max = 100;

  auto model = builder->build();
  utils::set_gripper_materials(*model);

  Eigen::VectorXd q0_robot(13), q_des(13 * 7);
  q0_robot << 0.887293, -0.305106, -0.941466, -1.88319, -0.247274, 1.69601, 0,
      0, 0, 0, 0, 0, 0;
  model->state.q0().head<13>() = q0_robot;
  model->state.q0().segment<3>(13) << 0.5, -0.1, 0.328001;
  model->state.q0().segment<4>(16) << 0, 0, 0, 1;

  constexpr double theta = 0.65;

  q_des << 0.887293, -0.305106, -0.941466, -1.88319, -0.247274, 1.69601, 0, 0,
      0, 0, 0, 0, 0, 0.864693, -0.297066, -0.956013, -2.03104, -0.251228,
      1.84777, 0, 0, 0, 0, 0, 0, 0, 0.864693, -0.297066, -0.956013, -2.03104,
      -0.251228, 1.84777, 0, theta, theta, -theta, -theta, -theta, theta,
      0.887839, -0.305098, -0.941652, -1.88168, -0.247257, 1.6946, 0, theta,
      theta, -theta, -theta, -theta, theta, 4.44089e-16, -0.195957,
      -4.45208e-16, -1.93015, 4.0453e-17, 1.7342, 0, theta, theta, -theta,
      -theta, -theta, theta, 4.44089e-16, -0.197213, -4.45819e-16, -1.99514,
      3.31717e-17, 1.79793, 0, theta, theta, -theta, -theta, -theta, theta,
      4.44089e-16, -0.177692, -4.5125e-16, -2.141, 1.3881e-17, 1.96331, 0,
      theta, theta, -theta, -theta, -theta, theta;

  constexpr double alpha = 0.5;
  q_des.segment(13 * 6, 7) =
      alpha * q_des.segment(13 * 6, 7) + (1 - alpha) * q_des.segment(13 * 5, 7);

  auto app = crisp::make_app(std::move(model));
  app->setControl([&](auto const &, auto const &data, auto control, auto) {
    update_control(control, data.sim.time, q0_robot, q_des);
  });

  app->init("Peg insertion", 1600, 900);
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
