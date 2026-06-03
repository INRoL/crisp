#include <algorithm>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <crisp/crisp.hpp>

std::vector<Eigen::VectorXd> read_csv_trajectory(const std::string &path,
                                                 int width) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("Could not open file: " + path);
  }

  std::vector<Eigen::VectorXd> trajectory;
  std::string line;
  while (std::getline(file, line)) {
    Eigen::VectorXd row(width);
    row.setZero();

    std::stringstream stream(line);
    std::string cell;
    int count = 0;
    while (std::getline(stream, cell, ',') && count < width) {
      row[count++] = std::stod(cell);
    }
    trajectory.push_back(row);
  }

  return trajectory;
}

std::vector<Eigen::Vector3d> xyz = {Eigen::Vector3d{0, 0, 0.333},
                                    Eigen::Vector3d{0, 0, 0},
                                    Eigen::Vector3d{0, -0.316, 0},
                                    Eigen::Vector3d{0.0825, 0, 0},
                                    Eigen::Vector3d{-0.0825, 0.384, 0},
                                    Eigen::Vector3d{0, 0, 0},
                                    Eigen::Vector3d{0.088, 0, 0},
                                    Eigen::Vector3d{0, 0, 0.107},
                                    Eigen::Vector3d{0, 0, 0}};
double PI = 3.1415;
std::vector<Eigen::Vector3d> rpy = {
    Eigen::Vector3d{0, 0, 0},       Eigen::Vector3d{-PI / 2, 0, 0},
    Eigen::Vector3d{PI / 2, 0, 0},  Eigen::Vector3d{PI / 2, 0, 0},
    Eigen::Vector3d{-PI / 2, 0, 0}, Eigen::Vector3d{PI / 2, 0, 0},
    Eigen::Vector3d{PI / 2, 0, 0},  Eigen::Vector3d{0, 0, 0},
    Eigen::Vector3d{0, 0, 0}};

std::vector<int> ax = {3, 3, 3, 3, 3, 3, 3, 0, 0};

Eigen::Matrix3d rotation(double theta, char axis) {
  Eigen::Matrix3d r;
  if (axis == 'x') {
    r << 1, 0, 0, 0, std::cos(theta), -std::sin(theta), 0, std::sin(theta),
        std::cos(theta);
  } else if (axis == 'y') {
    r << std::cos(theta), 0, std::sin(theta), 0, 1, 0, -std::sin(theta), 0,
        std::cos(theta);
  } else {
    r << std::cos(theta), -std::sin(theta), 0, std::sin(theta), std::cos(theta),
        0, 0, 0, 1;
  }
  return r;
}

Eigen::Vector3d unskew(const Eigen::Matrix3d &S) {
  return {S(2, 1), S(0, 2), S(1, 0)};
}

Eigen::Matrix4d get_rot(const Eigen::Vector3d &_rpy,
                        const Eigen::Vector3d &_xyz) {
  Eigen::Matrix3d R =
      rotation(_rpy[2], 'z') * rotation(_rpy[1], 'y') * rotation(_rpy[0], 'x');
  Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
  SE3.block(0, 0, 3, 3) = R;
  SE3.block(0, 3, 3, 1) = _xyz;
  return SE3;
}

Eigen::Matrix4d SE3_rot(const double th, const int ax) {
  Eigen::Matrix4d SE3_matrix = Eigen::Matrix4d::Identity();
  if (ax == 1) {
    SE3_matrix.block(0, 0, 3, 3) = rotation(th, 'x');
  } else if (ax == 2) {
    SE3_matrix.block(0, 0, 3, 3) = rotation(th, 'y');
  } else if (ax == 3) {
    SE3_matrix.block(0, 0, 3, 3) = rotation(th, 'z');
  }
  return SE3_matrix;
}

Eigen::Matrix4d get_trans(const Eigen::Vector3d &pos) {
  Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
  SE3.block(0, 3, 3, 1) = pos;
  return SE3;
}

Eigen::Matrix4d fk_ee(const Eigen::VectorXd &q, const Eigen::Matrix3d &R_base,
                      const Eigen::Vector3d &p_base) {
  std::vector<double> qt = {q(0), q(1), q(2), q(3), q(4), q(5), q(6), 0, 0};

  Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
  SE3.block(0, 0, 3, 3) = R_base;
  SE3.block(0, 3, 3, 1) = p_base;

  for (int i = 0; i < 9; ++i) {
    SE3 = SE3 * get_rot(rpy[i], xyz[i]) * SE3_rot(qt[i], ax[i]);
  }

  return SE3;
}

Eigen::MatrixXd jaco_ee(const Eigen::VectorXd &q, const Eigen::Matrix3d &R_base,
                        const Eigen::Vector3d &p_base) {
  std::vector<double> qt = {q(0), q(1), q(2), q(3), q(4), q(5), q(6), 0, 0};

  Eigen::Matrix4d SE3 = Eigen::Matrix4d::Identity();
  SE3.block(0, 0, 3, 3) = R_base;
  SE3.block(0, 3, 3, 1) = p_base;

  std::vector<Eigen::Matrix4d> T;
  T.push_back(SE3);

  for (int i = 0; i < 9; ++i) {
    SE3 = T[i] * get_rot(rpy[i], xyz[i]) * SE3_rot(qt[i], ax[i]);
    T.push_back(SE3);
  }

  Eigen::MatrixXd J(6, 7);
  J.setZero();
  for (int col = 0; col < 7; ++col) {
    Eigen::Vector3d tmp = T[col + 1].block(0, ax[col] - 1, 3, 1);
    Eigen::Vector3d tmp2 = SE3.block(0, 3, 3, 1) - T[col + 1].block(0, 3, 3, 1);
    tmp = tmp.cross(tmp2);
    J.block(0, col, 3, 1) = tmp;
    J.block(3, col, 3, 1) = T[col + 1].block(0, ax[col] - 1, 3, 1);
  }

  return J;
}

void ik_ee(Eigen::VectorXd &q, const Eigen::Matrix3d &R_base,
           const Eigen::Vector3d &p_base, const Eigen::Vector3d &p_t,
           const Eigen::Matrix3d &R_t, const Eigen::VectorXd &q_l,
           const Eigen::VectorXd &q_u, const Eigen::VectorXd &Kdiag) {
  Eigen::Matrix4d T;
  Eigen::MatrixXd J(6, 7);
  Eigen::Matrix3d R_e;
  Eigen::Vector3d p_e;
  Eigen::VectorXd dx(6);

  Eigen::VectorXd dq(q.size());

  Eigen::MatrixXd K(6, 6);
  K = Kdiag.asDiagonal();

  for (int i = 0; i < 300; ++i) {
    T = fk_ee(q, R_base, p_base);
    J = jaco_ee(q, R_base, p_base);
    R_e = T.block(0, 0, 3, 3);
    p_e = T.block(0, 3, 3, 1);

    dx.segment(0, 3) = p_e - p_t;
    dx.segment(3, 3) =
        R_e * unskew(0.5 * (R_t.transpose() * R_e - R_e.transpose() * R_t));

    dq = J.transpose() * K * dx;

    q = q - dq / 100;
    for (int j = 0; j < q.size(); ++j) {
      q[j] = std::max(std::min(q[j], q_u[j]), q_l[j]);
    }
  }
}

void my_control(Eigen::Ref<Eigen::VectorXd> act_in,
                const Eigen::VectorXd &q_cur, double time,
                const Eigen::VectorXd &q0, const Eigen::Matrix3d &R_base,
                const Eigen::Vector3d &p_base, const Eigen::Vector3d &p_t,
                const Eigen::Matrix3d &R_t, const Eigen::VectorXd &q_l,
                const Eigen::VectorXd &q_u) {
  act_in.setZero();

  if (time < 16) {
    Eigen::VectorXd Kdiag{{10, 10, 10, 10, 10, 10}};

    Eigen::VectorXd q = q_cur.segment(0, 7);
    ik_ee(q, R_base, p_base, p_t, R_t, q_l, q_u, Kdiag);
    act_in.segment(0, 7) = q0 + (q - q0) * std::min(time / 15, 1.0);
  } else {
    Eigen::VectorXd Kdiag{{1, 1, 10, 10, 10, 10}};

    Eigen::Matrix4d T = fk_ee(q_cur, R_base, p_base);
    Eigen::Vector3d p_t2 = T.block(0, 3, 3, 1) - Eigen::Vector3d{0, 0, 0.002};
    p_t2[0] = p_t[0];
    p_t2[1] = p_t[1];

    Eigen::Matrix3d R_t2 = R_t;

    Eigen::VectorXd q = q_cur.segment(0, 7);

    ik_ee(q, R_base, p_base, p_t2, R_t2, q_l, q_u, Kdiag);
    act_in.segment(0, 7) = q;
  }

  if (time > 17) {
    act_in(7) = q_cur[7] + PI / 6;
    if (q_cur[7] >= 4 * PI) {
      act_in(7) = q_cur[7];
    }
  }
}

double Fract(double x) { return x - std::floor(x); }
double Subtraction(double a, double b) { return std::max(a, -b); }
double Intersection(double a, double b) { return std::max(a, b); }
double Union(double a, double b) { return std::min(a, b); }

double bolt_distance(const Eigen::Vector3d &p, double radius_attr) {
  constexpr double screw = 1.0 / 0.005;
  const double radius = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  const double sqrt12 = std::sqrt(2.0) / 2.0;

  const double azimuth = std::atan2(p.y(), p.x());
  const double triangle =
      std::abs(Fract(p.z() * screw - azimuth / std::numbers::pi / 2.0) - 0.5);
  const double thread = (radius - triangle / screw) * sqrt12;

  double bolt = Subtraction(thread, 0.02 - std::abs(p.z() + 0.02));

  const double cone = (p.z() - radius) * sqrt12;

  bolt = Subtraction(bolt, cone + 1.0 * sqrt12);

  const double k = 6.0 / std::numbers::pi / 2.0;
  const double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  const double s0 = std::sin(ang);
  const double s1 = std::sin(ang + std::numbers::pi * 0.5);

  const double rx = s1 * p.x() - s0 * p.y();
  const double ry = s0 * p.x() + s1 * p.y();
  double head = rx - 0.028;

  head = Intersection(head, std::abs(p.z() + 0.0125) - 0.008);
  head = Intersection(head, (p.z() + radius - 0.22) * sqrt12);

  return Union(bolt, head);
}

Eigen::Vector3d bolt_gradient_fd(const Eigen::Vector3d &p, double radius_attr) {
  const double eps = 1e-8;
  const double d0 = bolt_distance(p, radius_attr);

  Eigen::Vector3d px = p;
  px.x() += eps;
  Eigen::Vector3d py = p;
  py.y() += eps;
  Eigen::Vector3d pz = p;
  pz.z() += eps;

  const double dx = (bolt_distance(px, radius_attr) - d0) / eps;
  const double dy = (bolt_distance(py, radius_attr) - d0) / eps;
  const double dz = (bolt_distance(pz, radius_attr) - d0) / eps;

  return Eigen::Vector3d(dx, dy, dz);
}

double nut_distance(const Eigen::Vector3d &p, double radius_attr) {
  constexpr double screw = 1 / 0.005;
  const double radius2 = std::sqrt(p.x() * p.x() + p.y() * p.y()) - radius_attr;
  const double sqrt12 = std::sqrt(2.0) / 2.0;

  const double azimuth = std::atan2(p.y(), p.x());
  const double triangle =
      std::abs(Fract(p.z() * screw - azimuth / std::numbers::pi / 2.0) - 0.5);
  const double thread2 = (radius2 - triangle / screw) * sqrt12;

  const double cone2 = (p.z() - radius2) * sqrt12;

  double hole = Subtraction(thread2, cone2 + 0.01 * sqrt12);

  const double k = 6.0 / std::numbers::pi / 2.0;
  const double ang = -std::floor(std::atan2(p.y(), p.x()) * k + 0.5) / k;

  const double s0 = std::sin(ang);
  const double s1 = std::sin(ang + std::numbers::pi * 0.5);

  const double rx = s1 * p.x() - s0 * p.y();
  const double ry = s0 * p.x() + s1 * p.y();

  double head = rx - 0.037;

  head = Intersection(head, std::abs(p.z() + 0.0075) - 0.0075);
  head = Intersection(head, (p.z() + radius2 - 0.22) * sqrt12);

  return Subtraction(head, hole);
}

Eigen::Vector3d nut_gradient_fd(const Eigen::Vector3d &p, double radius_attr) {
  const double eps = 1e-8;
  const double d0 = nut_distance(p, radius_attr);

  Eigen::Vector3d px = p;
  px.x() += eps;
  Eigen::Vector3d py = p;
  py.y() += eps;
  Eigen::Vector3d pz = p;
  pz.z() += eps;

  const double dx = (nut_distance(px, radius_attr) - d0) / eps;
  const double dy = (nut_distance(py, radius_attr) - d0) / eps;
  const double dz = (nut_distance(pz, radius_attr) - d0) / eps;

  return Eigen::Vector3d(dx, dy, dz);
}

int main() {
  auto builder = crisp::make_model_builder();

  builder->addRobot(
      {.path = "assets/panda/panda_hebi_demo.urdf", .root_pos = {0, 0, 0}});

  int bolt_sdf_type = crisp::register_sdf(
      [](crisp::Ref<crisp::Vector3r const> x,
         crisp::Ref<crisp::VectorXr const> p, int, crisp::real_t &phi,
         crisp::Ref<crisp::Vector3r> grad, crisp::Ref<crisp::Matrix3r>) -> int {
        const double radius_attr = static_cast<double>(p[0]);
        Eigen::Vector3d xd = x;

        const double dist = bolt_distance(xd, radius_attr);
        Eigen::Vector3d g = bolt_gradient_fd(xd, radius_attr);

        double gn = g.norm();
        Eigen::Vector3d n = Eigen::Vector3d::Zero();
        if (gn > 1e-12) {
          n = g / gn;
        }

        phi = static_cast<double>(dist);
        grad = n;
        return crisp::sdf_eval_grad;
      },
      [](crisp::Ref<crisp::Matrix3r const>, crisp::Ref<crisp::VectorXr const>,
         crisp::Ref<crisp::Vector6r> aabb) {
        aabb << -0.035, -0.035, -0.05, 0.035, 0.035, 0.05;
      },
      1);

  {
    auto &body = builder->addBody(
        {.pos = {0.5, -0.025, 0.015 - 0.011},
         .quat = {0, 1, 0, 0},
         .mass = 461.829,
         .inertia = {61.52395774, 38.72903532, 38.71283906, 0, 0, 0}},
        true);
    auto &geom = body.addGeom({.visual = true});
    double tol = 2000e-6;
    geom.createSDF({.type = bolt_sdf_type,
                    .param = crisp::to_view(
                        Eigen::VectorXd{{41.86565 / 1000 / 2 - tol / 2}})});
    geom.createMaterial({.rgba = crisp::oklch(0.74f, 0.02f, 255),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.45f, 0.45f, 0.45f},
                         .shininess = 64});
  }
  {
    // table
    auto &body = builder->addBody({.pos = {0.5, -0.025, 0}}, true);
    auto &geom = body.addGeom({.type = crisp::geometry_e::box,
                               .size = crisp::make_box_size(0.1, 0.1, 0.022),
                               .mu = 0.1});
    geom.createMaterial({.rgba = crisp::oklch(0.80f, 0.008f, 90),
                         .emission = {0.0f, 0.0f, 0.0f},
                         .specular = {0.04f, 0.04f, 0.04f},
                         .shininess = 16});
  }

  builder->vis().cam.target = {0.5f, -0.025f, 0.12f};
  builder->vis().cam.distance = 0.75f;
  builder->vis().cam.azimuth = 140.0f;
  builder->vis().cam.elevation = -18.0f;
  builder->vis().cam.fovy = 38.0f;
  builder->addLight({.type = crisp::light_e::directional,
                     .enabled = true,
                     .cast_shadow = true,
                     .pos = {3.0f, -2.0f, 1.0f},
                     .dir = {-3.0f, 2.0f, -1.0f},
                     .ambient = {0.1f, 0.1f, 0.1f},
                     .diffuse = {0.4f, 0.4f, 0.4f},
                     .specular = {0.5f, 0.5f, 0.5f},
                     .attenuation = {1.0f, 0.0f, 0.0f},
                     .cutoff = 0.0f,
                     .exponent = 0.0f});
  builder->addLight({.type = crisp::light_e::directional,
                     .enabled = true,
                     .cast_shadow = true,
                     .pos = {1.0f, -2.0f, 3.0f},
                     .dir = {-1.0f, 2.0f, -3.0f},
                     .ambient = {0.0f, 0.0f, 0.0f},
                     .diffuse = {0.8f, 0.8f, 0.8f},
                     .specular = {0.3f, 0.3f, 0.3f},
                     .attenuation = {1.0f, 0.0f, 0.0f},
                     .cutoff = 0.0f,
                     .exponent = 0.0f});
  for (auto joint :
       {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"}) {
    builder->addActuator(joint, {.gain = {0, 1000, 100}});
  }
  builder->addActuator("panda_joint9", {.gain = {0, 50, 5}});

  auto m = builder->build();
  Eigen::VectorXd q0{
      {0.00000, -0.06313, 0.00000, -2.18291, -0.00000, 2.12035, -0.00000, 0}};
  m->state.q0() = q0;
  m->act.u0() << 0, 0, 0, 0, 0, 0, 0, 0;

  Eigen::VectorXd q_l{{-PI, -PI, -PI, -PI, -PI, -PI, -1e+10, -1e+10}};
  Eigen::VectorXd q_u{{PI, PI, PI, PI, PI, PI, 1e+10, 1e+10}};

  m->opt.gravity.setZero();

  m->opt.sol.max_iter = 100;
  m->opt.sol.type = crisp::solver_e::canal;

  auto app = crisp::make_app(std::move(m));

  auto u_traj = read_csv_trajectory("assets/boltnut/boltnut_q_13.csv", 8);
  app->data().act.u() = u_traj[0];

  app->setControl([&](auto const &m, auto const &d, auto u, auto udot) {
    my_control(u, d.state.q(), d.sim.time, q0, Eigen::Matrix3d::Identity(),
               Eigen::Vector3d{0, 0, 0.011},
               Eigen::Vector3d{0.5 + 0.00, -0.025 + 0.00, 0.015 + 0.155},
               rotation(PI, 'x'), q_l, q_u);
  });

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
