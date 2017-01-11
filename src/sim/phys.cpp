#include "sim/scene.h"
#include <valarray>
using namespace std;

namespace {

struct BodyState {
  dvec3 pos;
  dquat rot;
  dvec3 momentum;
  dvec3 ang;

  BodyState() = default;
  BodyState(const Body& b): pos(b.pos), rot(b.rot), momentum(b.momentum), ang(b.ang) {}
  BodyState(const double* p) {
    memcpy(this, p, sizeof(*this));
  }

  void ToBody(Body& b) {
    b.pos = pos; b.rot = rot; b.momentum = momentum; b.ang = ang;
  }
  void ToArray(double* p) {
    memcpy(p, this, sizeof(*this));
  }
};

struct BodyForce {
  dvec3 force = {0, 0, 0};
  dvec3 torque = {0, 0, 0};
};

void RungeKutta4(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f) {
  valarray<double> k1(y.size()), k2(y.size()), k3(y.size()), k4(y.size());
  f(y, k1);
  f(y + h/2*k1, k2);
  f(y + h/2*k2, k3);
  f(y + h*k3, k4);
  y += h/6*k1 + h/3*k2 + h/3*k3 + h/6*k4;
}//*/

/*
// If you want to see the difference between first-order and fourth-order integration,
// try using Euler() instead of RungeKutta4() (also fewer substeps).
// Torque-free precession of this single rotating box degrade in a few seconds with Euler():
// scene.AddBody(MakeBox(dvec3(.2, .1, .3)).MultiplyMass(2700))->ang = dvec3(0,-1.24991,-0.758193);
void Euler(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f) {
  valarray<double> k(y.size());
  f(y, k);
  y += h*k;
}//*/

} // namespace {

void Scene::PhysicsStep(double dt) {
  vector<BodyForce> forces(bodies.size());
  const size_t stride = sizeof(BodyState) / sizeof(double);
  valarray<double> state_vec(bodies.size() * stride);
  size_t i = 0;
  for (Body& body: bodies) {
    for (const auto& f: body.forces) {
      forces[i].force += f.second;
      forces[i].torque += f.first.Cross(f.second);
    }
    BodyState(body).ToArray(&state_vec[i * stride]);
  }
  auto f = [&](const valarray<double>& y, valarray<double>& yp) {
    size_t i = 0;
    for (Body& body: bodies) {
      BodyState s(&y[i * stride]);
      BodyState p;
      p.pos = s.momentum / body.mass;
      dvec3 av = body.inv_inertia * s.rot.Untransform(s.ang);
      p.rot = s.rot * fquat(0, av.x, av.y, av.z) * .5;
      p.momentum = forces[i].force;
      p.ang = forces[i].torque;
      p.ToArray(&yp[i * stride]);
      ++i;
    }
  };
  const int steps = 100;
  for (int i = 0; i < steps; ++i) {
    RungeKutta4(state_vec, dt / steps, f);
  }
  i = 0;
  for (Body& body: bodies) {
    BodyState(&state_vec[i * stride]).ToBody(body);
    body.rot.NormalizeMe();
    ++i;
  }
}
