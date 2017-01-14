#include "sim/scene.h"
#include <valarray>
#include <cassert>
#include <iostream>
using namespace std;

namespace {

static Body fixed_body = Body(-1);

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

// According to [1], this method has only second order accuracy for rotations.
// Still it seems to perform somewhat better than MidpointMethod() in my experiments.
// [1] http://euclid.ucsd.edu/~sbuss/ResearchWeb/accuraterotation/paper.pdf
void RungeKutta4(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f) {
  valarray<double> k1(y.size()), k2(y.size()), k3(y.size()), k4(y.size());
  f(y, k1);
  f(y + h/2*k1, k2);
  f(y + h/2*k2, k3);
  f(y + h*k3, k4);
  y += h/6*k1 + h/3*k2 + h/3*k3 + h/6*k4;
}//*/

/*
void MidpointMethod(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f) {
  valarray<double> k1(y.size()), k2(y.size());
  f(y, k1);
  f(y + h/2*k1, k2);
  y += h*k2;
}//*/

/*
// If you want to see the difference between first-order and second-order integration,
// try using Euler() instead of RungeKutta4() (also fewer substeps).
// Torque-free precession of this single rotating box degrade in a few seconds with Euler():
// scene.AddBody(MakeBox(dvec3(.2, .1, .3)).MultiplyMass(2700))->ang = dvec3(0,-1.24991,-0.758193);
void Euler(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f) {
  valarray<double> k(y.size());
  f(y, k);
  y += h*k;
}//*/

// Prevent errors from accumulating by coercing the bodies into meeting all constraints,
// together with their first derivative, in a physically incorrect way. Done after a normal update step.
void EnforceConstraints(Scene& scene) {
  for (const Constraint& c: scene.constraints) {
    Body& b1 = c.body1 == -1 ? fixed_body : scene.bodies[c.body1];
    Body& b2 = scene.bodies[c.body2];

    auto clear_vec = [&](dvec3& v, Constraint::dof_t d) {
      double l = 0;
      d &= c.lock;
      if (d & (Constraint::DOF::PX | Constraint::DOF::RX)) { l += v.x*v.x; v.x = 0; }
      if (d & (Constraint::DOF::PY | Constraint::DOF::RY)) { l += v.y*v.y; v.y = 0; }
      if (d & (Constraint::DOF::PZ | Constraint::DOF::RZ)) { l += v.z*v.z; v.z = 0; }
      return l;
    };

    // Rotation.
    {
      dquat q = c.rot1 * b1.rot.Conjugate() * b2.rot * c.rot2.Conjugate();
      double l = 0;
      if (c.lock & Constraint::DOF::RX) { l += q.b*q.b; q.b = 0; }
      if (c.lock & Constraint::DOF::RY) { l += q.c*q.c; q.c = 0; }
      if (c.lock & Constraint::DOF::RZ) { l += q.d*q.d; q.d = 0; }
      q.a = (q.a < 0 ? -1 : 1) * sqrt(q.a*q.a + l); // avoid gimbal lock
      b2.rot = b1.rot * c.rot1.Conjugate() * q * c.rot2;
      scene.leaked_rotation += sqrt(l);
    }

    // Translation.
    {
      dvec3 p = c.rot1.Transform(b1.rot.Untransform(b2.rot.Transform(c.pos2) + b2.pos - b1.pos) - c.pos1);
      double l = clear_vec(p, Constraint::DOF::POS);
      b2.pos = b1.rot.Transform(c.rot1.Untransform(p) + c.pos1) + b1.pos - b2.rot.Transform(c.pos2);
      scene.leaked_translation += sqrt(l);
    }

    // Angular velocity.
    {
      dvec3 w = c.rot1.Transform(b1.rot.Untransform(b2.rot.Transform(b2.inv_inertia * b2.rot.Untransform(b2.ang))) -
                                 b1.inv_inertia * b1.rot.Untransform(b1.ang));
      double l = clear_vec(w, Constraint::DOF::ROT);
      b2.ang = b2.rot.Transform(b2.inv_inertia.Inverse() *
                                b2.rot.Untransform(b1.rot.Transform(c.rot1.Untransform(w) +
                                                                    b1.inv_inertia * b1.rot.Untransform(b1.ang))));
      scene.leaked_angular_velocity += sqrt(l);
    }

    // Linear velocity.
    {
      dvec3 rot_vel =
        b1.rot.Untransform(b2.rot.Transform((b2.inv_inertia *
                                             b2.rot.Untransform(b2.ang)).Cross(c.pos2))) -
        (b1.inv_inertia * b1.rot.Untransform(b1.ang)).Cross(c.pos1);
      dvec3 v = c.rot1.Transform(rot_vel +
                                 b1.rot.Untransform(b2.momentum/b2.mass - b1.momentum/b1.mass));
      double l = clear_vec(v, Constraint::DOF::POS);
      b2.momentum = (b1.rot.Transform(c.rot1.Untransform(v) - rot_vel) + b1.momentum/b1.mass) * b2.mass;
      scene.leaked_velocity += sqrt(l);
    }
  }
}

} // namespace {

double Scene::GetEnergy() const {
  double r = 0;
  for (const Body& b: bodies)
    r += .5 * (b.momentum.LengthSquare() / b.mass + b.ang.Dot(b.rot.Transform(b.inv_inertia * b.rot.Untransform(b.ang))));
  return r;
}

void Scene::PhysicsStep(double dt) {
  vector<BodyForce> forces(bodies.size());
  const size_t stride = sizeof(BodyState) / sizeof(double);
  valarray<double> state_vec(bodies.size() * stride);
  for (size_t i = 0; i < bodies.size(); ++i) {
    Body& body = bodies[i];
    for (const auto& f: body.forces) {
      forces[i].force += f.second;
      forces[i].torque += f.first.Cross(f.second);
    }
    BodyState(body).ToArray(&state_vec[i * stride]);
  }
  auto f = [&](const valarray<double>& y, valarray<double>& yp) {
    for (size_t i = 0; i < bodies.size(); ++i) {
      Body& body = bodies[i];
      BodyState s(&y[i * stride]);
      BodyState p;
      p.pos = s.momentum / body.mass;
      dvec3 av = body.inv_inertia * (s.rot.ToMatrix().Transposed() * s.ang);
      p.rot = s.rot * fquat(0, av.x, av.y, av.z) * .5;
      p.momentum = forces[i].force;
      p.ang = forces[i].torque;
      p.ToArray(&yp[i * stride]);
    }
  };
  const int steps = 100;
  for (int i = 0; i < steps; ++i) {
    RungeKutta4(state_vec, dt / steps, f);
  }
  for (size_t i = 0; i < bodies.size(); ++i) {
    Body& body = bodies[i];
    BodyState(&state_vec[i * stride]).ToBody(body);
    body.rot.NormalizeMe();
  }
  EnforceConstraints(*this);
}

Constraint* Scene::AddConstraint(int body1, int body2, dvec3 pos2, dquat rot2, Constraint::dof_t lock) {
  assert(body1 >= -1);
  assert(body1 < (int)bodies.size());
  assert(body2 >= 0);
  assert(body2 < (int)bodies.size());

  Body& b1 = body1 == -1 ? fixed_body : bodies[body1];
  Body& b2 = bodies[body2];

  Constraint c;
  c.lock = lock;
  c.body1 = body1;
  c.body2 = body2;
  c.pos2 = pos2;
  c.rot2 = rot2;
  c.pos1 = b1.rot.Untransform((b2.rot.Transform(pos2) + b2.pos) - b1.pos);
  c.rot1 = rot2 * b2.rot.Conjugate() * b1.rot;

  constraints.push_back(c);
  return &constraints.back();
}
