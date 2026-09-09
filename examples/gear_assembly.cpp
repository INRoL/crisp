#include <crisp/crisp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <span>
#include <tuple>
#include <vector>

#include "common/materials.hpp"
#include "common/views.hpp"

namespace {

constexpr Eigen::Index kControlSize = 13;
constexpr double kFiniteDifferenceStep = 1e-8;

double sdf_intersection(double a, double b) { return std::max(a, b); }

double sdf_subtraction(double a, double b) { return std::max(a, -b); }

double circle_sdf(double radial_distance, double radius) {
  return radial_distance - radius;
}

double smooth_union(double a, double b, double smoothing) {
  const double blend =
      std::min(std::max(0.5 + 0.5 * (b - a) / smoothing, 0.0), 1.0);
  return b * (1.0 - blend) + a * blend - smoothing * blend * (1.0 - blend);
}

double smooth_intersection(double a, double b, double smoothing) {
  return sdf_subtraction(
      sdf_intersection(a, b),
      smooth_union(sdf_subtraction(a, b), sdf_subtraction(b, a), smoothing));
}

double positive_modulo(double value, double modulus) {
  return value - modulus * std::floor(value / modulus);
}

double gear_distance_2d(const Eigen::Vector3d &point, double rotation,
                        double pitch_diameter, double tooth_count,
                        double bore_diameter) {
  const double pressure_angle =
      3.096e-5 * tooth_count * tooth_count - 6.557e-3 * tooth_count + 0.551;
  const double pitch_radius = pitch_diameter / 2.0;
  const double radial_distance =
      std::sqrt(point[0] * point[0] + point[1] * point[1]);
  const double diametral_pitch = tooth_count / pitch_diameter;
  const double circular_pitch = std::numbers::pi / diametral_pitch;
  const double addendum = 1.0 / diametral_pitch;
  const double outer_radius = (pitch_diameter + 2.0 * addendum) / 2.0;
  const double tooth_height = 2.2 / diametral_pitch;
  const double bore_radius = bore_diameter / 2.0;

  if (bore_radius - radial_distance > 0.0) {
    return bore_radius - radial_distance;
  }
  if (outer_radius - radial_distance < -0.2 * pitch_diameter) {
    return radial_distance - outer_radius;
  }

  const double base_radius = pitch_diameter * std::cos(pressure_angle) / 2.0;
  const double angle = std::atan2(point[1], point[0]) + rotation;
  const double tooth_angle = circular_pitch / pitch_radius;
  const double involute_angle = std::acos(base_radius / pitch_radius);
  const double involute_offset = std::tan(involute_angle) - involute_angle;
  const double shift = tooth_angle / 2.0 - 2.0 * involute_offset;
  const double angle_a =
      positive_modulo(angle + shift / 2.0, tooth_angle) - shift / 2.0;
  const double angle_b =
      positive_modulo(-angle - shift + shift / 2.0, tooth_angle) - shift / 2.0;

  double dista = -1.0e6;
  double distb = -1.0e6;
  if (base_radius < radial_distance) {
    const double acos_base_radius = std::acos(base_radius / radial_distance);
    const double tangent_length = std::sqrt(radial_distance * radial_distance -
                                            base_radius * base_radius);
    dista = tangent_length - base_radius * (angle_a + acos_base_radius);
    distb = tangent_length - base_radius * (angle_b + acos_base_radius);
  }

  const double outer = circle_sdf(radial_distance, outer_radius);
  const double root = circle_sdf(radial_distance, outer_radius - tooth_height);
  const double bore = circle_sdf(radial_distance, bore_radius);
  double teeth = sdf_intersection(dista, distb);
  const double base_walls = sdf_intersection(angle_a - (tooth_angle - shift),
                                             angle_b - (tooth_angle - shift));

  teeth = sdf_intersection(base_walls, teeth);
  teeth = smooth_intersection(outer, teeth, 0.0035 * pitch_diameter);
  teeth = smooth_union(root, teeth, base_radius - outer_radius + tooth_height);
  teeth = sdf_subtraction(teeth, bore);

  return teeth;
}

double extrude_sdf(double p2, double sdf_2d, double half_h) {
  const double planar_distance = sdf_2d;
  const double axial_distance = std::abs(p2) - half_h;
  const double planar_outside = std::max(planar_distance, 0.0);
  const double axial_outside = std::max(axial_distance, 0.0);
  return std::min(std::max(planar_distance, axial_distance), 0.0) +
         std::sqrt(planar_outside * planar_outside +
                   axial_outside * axial_outside);
}

double hollow_cylinder_sdf(const Eigen::Vector3d &point, double outer_radius,
                           double inner_radius, double half_height) {
  const double radial_distance =
      std::sqrt(point[0] * point[0] + point[1] * point[1]);
  const double outer =
      extrude_sdf(point[2], radial_distance - outer_radius, half_height);
  const double inner_hole = inner_radius - radial_distance;
  return sdf_intersection(outer, inner_hole);
}

double gear_with_hub_distance(const Eigen::Vector3d &point, double rotation,
                              double pitch_diameter, double tooth_count,
                              double gear_thickness, double hole_diameter,
                              double hub_radius, double hub_height,
                              double hub_center) {
  const Eigen::Vector3d gear_point{point[0], point[2], point[1]};
  const double gear =
      extrude_sdf(gear_point[2] - 0.5 * gear_thickness,
                  gear_distance_2d(gear_point, rotation, pitch_diameter,
                                   tooth_count, hole_diameter),
                  0.5 * gear_thickness);

  const Eigen::Vector3d hub_point{point[0], point[2], point[1] - hub_center};
  const double hub = hollow_cylinder_sdf(hub_point, hub_radius,
                                         0.5 * hole_diameter, 0.5 * hub_height);

  return std::min(gear, hub);
}

std::tuple<double, Eigen::Vector3d>
gear_distance_and_gradient(const Eigen::Vector3d &point,
                           crisp::Ref<crisp::VectorXr const> param) {
  auto dist_at = [&](Eigen::Vector3d const &x) {
    return gear_with_hub_distance(x, param[7], param[0], param[1], param[2],
                                  param[3], param[4], param[5], param[6]);
  };

  const double distance = dist_at(point);
  Eigen::Vector3d grad;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d displaced = point;
    displaced[axis] += kFiniteDifferenceStep;
    grad[axis] = (dist_at(displaced) - distance) / kFiniteDifferenceStep;
  }
  if (grad.norm() > 1e-12) {
    grad.normalize();
  } else {
    grad.setZero();
  }
  return {distance, grad};
}

int evaluate_gear_sdf(crisp::Ref<crisp::Vector3r const> x_rel,
                      crisp::Ref<crisp::VectorXr const> param, int requested,
                      crisp::real_t &phi, crisp::Ref<crisp::Vector3r> grad,
                      crisp::Ref<crisp::Matrix3r>) {
  if (!(requested & crisp::sdf_eval_grad)) {
    phi = gear_with_hub_distance(x_rel, param[7], param[0], param[1], param[2],
                                 param[3], param[4], param[5], param[6]);
    return 0;
  }
  std::tie(phi, grad) = gear_distance_and_gradient(x_rel, param);
  return crisp::sdf_eval_grad;
}

void compute_gear_aabb(crisp::Ref<crisp::VectorXr const> param,
                       crisp::Ref<crisp::Vector6r> aabb) {
  const double pitch_diameter = param[0];
  const double tooth_count = param[1];
  const double outer_radius =
      (pitch_diameter + 2.0 * pitch_diameter / tooth_count) / 2.0;
  const double radius = std::max(outer_radius, static_cast<double>(param[4]));
  const double y_max = std::max(static_cast<double>(param[2]),
                                static_cast<double>(param[6] + 0.5 * param[5]));
  aabb << -radius, -0.001, -radius, radius, y_max + 0.001, radius;
}

struct ControlLeg {
  double end_time;
  double ramp_duration;
};

void update_control(Eigen::Ref<Eigen::VectorXd> act_in, double time,
                    const Eigen::VectorXd &q_0, const Eigen::VectorXd &q_des,
                    std::span<const ControlLeg> legs) {
  Eigen::VectorXd q_i = q_0;
  double t_i = 0.0;
  for (std::size_t k = 0; k < legs.size(); ++k) {
    Eigen::VectorXd q_f = q_des.segment(kControlSize * k, kControlSize);
    if (time < legs[k].end_time) {
      const double progress =
          std::min((time - t_i) / legs[k].ramp_duration, 1.0);
      act_in = q_i + (q_f - q_i) * progress;
      return;
    }
    t_i = legs[k].end_time;
    q_i = q_f;
  }
  act_in = q_i;
}

} // namespace

int main() {
  constexpr double pla_mu = 0.35;
  constexpr double pla_k = 1e5;
  constexpr double gear_bore_diameter = 0.0105;

  const int gear_sdf_type =
      crisp::register_sdf({.name = "gear_assembly/gear_with_hub",
                           .eval = evaluate_gear_sdf,
                           .aabb = compute_gear_aabb,
                           .nparam = 8});

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
    auto &body = builder->addBody({.name = "gear_base",
                                   .pos = {0.4797, 0.0, 0.3},
                                   .mass = 0.02,
                                   .inertia = {0.001, 0.001, 0.001, 0, 0, 0}});
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh({.path = "assets/industrealkit/gear_base.obj"});
    auto &collision_geom =
        body.addGeom({.visual = false, .mu = pla_mu, .k = pla_k});
    collision_geom.createMesh(
        {.path = "assets/industrealkit/gear_base_remesh1.5mm_upper.obj"});
  }

  builder->addExclude("gear_base", "left_inner_finger");
  builder->addExclude("gear_base", "right_inner_finger");

  {
    auto &body = builder->addBody(
        {.pos = {0.4495, 0.0, 0.305},
         .quat = Eigen::Quaterniond(
             Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitX())),
         .ipos = {0, 0.009047, 0},
         .mass = 0.04991,
         .inertia = {1.1442e-05, 1.8314e-05, 1.1460e-05, 0, 0, 0}},
        false);
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh({.path = "assets/industrealkit/gear_large.obj"});
    visual_geom.createMaterial({.rgba = crisp::oklch(0.70f, 0.060f, 255.0f),
                                .specular = {0.25f, 0.25f, 0.25f},
                                .shininess = 48.0f});
    auto &collision_geom =
        body.addGeom({.visual = false, .mu = pla_mu, .k = pla_k});
    collision_geom.createSDF({.type = gear_sdf_type,
                              .param = crisp::to_view(Eigen::VectorXd{
                                  {0.060, 60.0, 0.010, gear_bore_diameter,
                                   0.0175, 0.025, 0.0125, 0.0328649}})});
  }

  {
    auto &body = builder->addBody(
        {.pos = {0.5305, 0.0, 0.305},
         .quat = Eigen::Quaterniond(
             Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitX())),
         .ipos = {0, 0.010432, 0},
         .mass = 0.004823,
         .inertia = {3.8341e-07, 2.7299e-07, 3.8467e-07, 0, 0, 0}},
        false);
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh({.path = "assets/industrealkit/gear_small.obj"});
    visual_geom.createMaterial({.rgba = crisp::oklch(0.70f, 0.065f, 155.0f),
                                .specular = {0.25f, 0.25f, 0.25f},
                                .shininess = 48.0f});
    auto &collision_geom =
        body.addGeom({.visual = false, .mu = pla_mu, .k = pla_k});
    collision_geom.createSDF(
        {.type = gear_sdf_type,
         .param = crisp::to_view(
             Eigen::VectorXd{{0.020, 20.0, 0.010, gear_bore_diameter, 0.0080,
                              0.025, 0.0125, 0.10763560000000001}})});
  }

  {
    auto &body = builder->addBody(
        {.pos = {0.5, 0.0, 0.305},
         .quat = Eigen::Quaterniond(
             Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitX())),
         .ipos = {0, 0.010545, 0},
         .mass = 0.02577,
         .inertia = {3.5781e-06, 4.5069e-06, 3.5892e-06, 0, 0, 0}},
        false);
    auto &visual_geom = body.addGeom({.con_type = 0, .con_affinity = 0});
    visual_geom.createMesh({.path = "assets/industrealkit/gear_medium.obj"});
    visual_geom.createMaterial({.rgba = crisp::oklch(0.70f, 0.070f, 75.0f),
                                .specular = {0.25f, 0.25f, 0.25f},
                                .shininess = 48.0f});
    auto &collision_geom =
        body.addGeom({.visual = false, .mu = pla_mu, .k = pla_k});
    collision_geom.createSDF(
        {.type = gear_sdf_type,
         .param = crisp::to_view(
             Eigen::VectorXd{{0.040, 40.0, 0.010, gear_bore_diameter, 0.0150,
                              0.025, 0.0125, 0.052789800000000012}})});
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
  builder->opt().sol.type = crisp::solver_e::sub_admm;
  builder->opt().sol.max_iter = 200;
  builder->cap().ncon_max = 200;
  builder->cap().nthread = 0;

  auto model = builder->build();
  utils::set_gripper_materials(*model);

  Eigen::VectorXd q0_robot(13), q_des(13 * 13);
  q0_robot << 0.887293, -0.305106, -0.941466, -1.88319, -0.247274, 1.69601, 0,
      0, 0, 0, 0, 0, 0;
  model->state.q0().head<13>() = q0_robot;

  const Eigen::Quaterniond gear_start =
      Eigen::AngleAxisd(std::numbers::pi / 40, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(std::numbers::pi / 2, Eigen::Vector3d::UnitX());
  model->state.q0().segment<3>(27) << 0.5, -0.1, 0.3;
  model->state.q0().segment<4>(30) << gear_start.x(), gear_start.y(),
      gear_start.z(), gear_start.w();

  constexpr double theta = 0.50;
  constexpr double theta_open = 0.30;

  constexpr std::array<double, 7> arm_approach{
      0.866253587519653,   -0.303004161845923, -0.939577461500201,
      -1.96070549496487,   -0.248401744336169, 1.77206960882008,
      -0.00029644439392559};
  constexpr std::array<double, 7> arm_grasp{
      0.867387713392193,   -0.280256982357164, -0.979053892659493,
      -2.09574044399504,   -0.247403477913725, 1.92633483493197,
      -0.00541095127292980};
  constexpr std::array<double, 7> arm_lift{
      0.865319488813745,   -0.302155326199006, -0.944396677767761,
      -1.98508970486715,   -0.249999089675956, 1.79734444275670,
      0.000384893136463172};

  constexpr std::array<double, 7> arm_move{
      0.0, -0.198801867733514, 0.0, -2.02796841600692,
      0.0, 1.82916654868032,   0.0};
  constexpr std::array<double, 7> arm_contact{
      0.0, -0.192963848643472, 0.0, -2.08441933515932,
      0.0, 1.89145548850511,   0.0};

  constexpr std::array<double, 7> arm_engage{
      0.0, -0.184487345203457, 0.0, -2.12814965871734,
      0.0, 1.94366231351389,   0.0};

  constexpr std::array<double, 7> arm_hold{
      0.0, -0.183212383969216, 0.0, -2.11391359776781,
      0.0, 1.93070121542153,   0.0};

  constexpr double turn = 0.5;

  std::vector<ControlLeg> legs;
  legs.reserve(13);
  auto add_leg = [&, index = 0](const std::array<double, 7> &arm, double grip,
                                double twist, ControlLeg window) mutable {
    q_des.segment<7>(kControlSize * index) =
        Eigen::Map<Eigen::Matrix<double, 7, 1> const>(arm.data());
    q_des[kControlSize * index + 6] += twist;
    q_des.segment<6>(kControlSize * index + 7) << grip, grip, -grip, -grip,
        -grip, grip;
    legs.push_back(window);
    ++index;
  };

  add_leg(arm_approach, 0, 0, {2.0, 1.5});
  add_leg(arm_grasp, 0, 0, {3.5, 1.2});
  add_leg(arm_grasp, theta, 0, {5.0, 1.2});
  add_leg(arm_lift, theta, 0, {6.5, 1.2});
  add_leg(arm_move, theta, 0, {8.5, 1.5});
  add_leg(arm_contact, theta, 0, {10.0, 1.2});
  add_leg(arm_engage, theta, 0, {11.2, 1.0});
  add_leg(arm_engage, theta_open, 0, {11.8, 0.3});
  add_leg(arm_hold, theta_open, 0, {13.0, 1.0});
  add_leg(arm_hold, theta, 0, {14.0, 0.5});
  add_leg(arm_hold, theta, turn, {16.5, 2.0});
  add_leg(arm_hold, theta, -turn, {19.0, 2.0});
  add_leg(arm_hold, theta, 0, {20.5, 1.0});

  auto app = crisp::make_app(std::move(model));
  app->setControl([&](auto const &, auto const &data, auto control, auto) {
    update_control(control, data.sim.time, q0_robot, q_des, legs);
  });

  app->init("Gear assembly", 1600, 900);
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
