#include <crisp/crisp.hpp>

int main() {
  auto builder = crisp::make_model_builder();

  // robots
  int N = 2;
  for (int i = 0; i < N * N * N; ++i) {
    builder->addRobot({.path = "assets/laikago/laikago.urdf"}, false);
  }

  builder->vis().cam.target = {-0.45f, -0.2f, 0.2f};
  builder->vis().cam.distance = 2.4f;
  builder->vis().cam.azimuth = 135.0f;
  builder->vis().cam.elevation = -22.0f;
  builder->vis().cam.fovy = 42.0f;
  builder->addLight({.type = crisp::light_e::directional,
                     .enabled = true,
                     .cast_shadow = true,
                     .pos = {2.0f, -3.0f, 4.0f},
                     .dir = {-2.0f, 3.0f, -4.0f},
                     .ambient = {0.25f, 0.25f, 0.25f},
                     .diffuse = {0.75f, 0.75f, 0.75f},
                     .specular = {0.45f, 0.45f, 0.45f},
                     .attenuation = {1.0f, 0.0f, 0.0f},
                     .cutoff = 0.0f,
                     .exponent = 0.0f});
  builder->opt().con.erp = 0.01;
  builder->opt().sol.type = crisp::solver_e::sub_admm;
  builder->cap().ncon_max = 200;

  auto m = builder->build();

  m->state.q0().setZero();
  for (int i = 0; i < m->size.nlim; ++i) {
    int qadr = m->lim.qadr[i];
    m->state.q0[qadr] = (m->lim.range[i][0] + m->lim.range[i][1]) / 2.0;
  }
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      for (int k = 0; k < N; ++k) {
        int ind = N * N * i + N * j + k;
        m->state.q0().segment(19 * ind, 7) << 0.9 * (i - 1) + 0 * k + 0,
            0.4 * (j - 1) + 0.1 * k, 0.4 * (k + 1), 0, 0, 0, 1;
      }
    }
  }

  auto app = crisp::make_app(std::move(m));

  app->init();
  while (app->isOpen()) {
    app->render();
  }

  return 0;
}
