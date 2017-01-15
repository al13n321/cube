#include "sim/scene.h"
#include "util/linear.h"
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

  void FromBody(const Body& b) {
    pos = b.pos; rot = b.rot; momentum = b.momentum; ang = b.ang;
  }
  void ToBody(Body& b) {
    b.pos = pos; b.rot = rot; b.momentum = momentum; b.ang = ang;
  }
};

struct BodyForce {
  dvec3 force = {0, 0, 0};
  dvec3 torque = {0, 0, 0};
};

struct StateVector {
  StateVector(size_t n): bodies_(n) {}

  size_t size() const {
    return bodies_.size();
  }
  BodyState& operator[](size_t i) {
    return bodies_[i];
  }
  const BodyState& operator[](size_t i) const {
    return bodies_[i];
  }

  // *this = s + h * v;  s point to *this
  StateVector& AddMul(const StateVector& s, double h, const StateVector& v) {
    assert(size() == v.size());
    assert(size() == s.size());
    for (size_t i = 0; i < size(); ++i) {
      bodies_[i].pos = s[i].pos + v[i].pos * h;
      bodies_[i].rot = s[i].rot + v[i].rot * h;
      bodies_[i].momentum = s[i].momentum + v[i].momentum * h;
      bodies_[i].ang = s[i].ang + v[i].ang * h;
    }
    return *this;
  }

 private:
  vector<BodyState> bodies_;
};

struct Context {
  vector<BodyForce> external_forces;
  // External + constraint.
  vector<BodyForce> effective_forces;
  // Constraint idx -> idx of the first var. #vars is # locked DOFs.
  vector<int> var_idx;
  // body_idx*6+f -> coefs of linear combination of vars that is equal
  // to force #f on the body; f: 0-2 - linear force xyz, 3-5 - torque xyz.
  DMatrix force_from_vars;
  // Linear equation system. Vars - forces/torques from constraints,
  // rows - constraints (second derivative), last column - "b" as in Ax=b.
  DMatrix equations;
};

// According to [1], this method has only second order accuracy for rotations.
// Still it seems to perform somewhat better than MidpointMethod() in my experiments.
// [1] http://euclid.ucsd.edu/~sbuss/ResearchWeb/accuraterotation/paper.pdf
void RungeKutta4(StateVector& y, double h, function<void(const StateVector& y, StateVector& yp)> f) {
  StateVector k1(y.size()), k2(y.size()), k3(y.size()), k4(y.size()), ty(y.size());
  f(y, k1);
  f(ty.AddMul(y, h/2, k1), k2);
  f(ty.AddMul(y, h/2, k2), k3);
  f(ty.AddMul(y, h, k3), k4);
  y.AddMul(y, h/6, k1);
  y.AddMul(y, h/3, k2);
  y.AddMul(y, h/3, k3);
  y.AddMul(y, h/6, k4);
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

// Fills context.effective_forces.
void ResolveForces(Scene& scene, const StateVector& state, Context& context) {
  size_t nvars = context.var_idx.back();
  auto& fv = context.force_from_vars;
  fv.Resize(scene.bodies.size() * 6, nvars + 1);
  fv.Fill(0);

  for (size_t i = 0; i < scene.bodies.size(); ++i) {
    context.external_forces[i].force.ToArray(fv[i*6] + nvars, fv.Stride());
    context.external_forces[i].torque.ToArray(fv[i*6 + 3] + nvars, fv.Stride());
  }

  auto mat_to_vars = [&](const dmat3& m, size_t fi, size_t var, Constraint::dof_t dofs) {
    for (size_t i = 0; i < 3; ++i) {
      size_t v = var;
      for (size_t j = 0; j < 3; ++j) {
        if (dofs & ((Constraint::DOF::PX | Constraint::DOF::RX) << j))
          fv[fi + i][v++] += m[i][j];
      }
    }
  };
  for (size_t i = 0; i < scene.constraints.size(); ++i) {
    const Constraint& c = scene.constraints[i];
    const BodyState& s1 = state[c.body1];
    const BodyState& s2 = state[c.body2];
    size_t var = context.var_idx[i];
    auto pos_dof = c.lock & Constraint::DOF::POS;
    if (pos_dof) {
      dmat3 m = (s1.rot * c.rot1.Conjugate()).ToMatrix();
      mat_to_vars(-m, c.body1*6, var, pos_dof);
      mat_to_vars(m, c.body2*6, var, pos_dof);
      // Body torque depends on constraint force too (not only on constraint torque).
      m = -s1.rot.ToMatrix() * c.pos1.Skew() * c.rot1.Conjugate().ToMatrix();
      mat_to_vars(m, c.body1*6 + 3, var, pos_dof);
      m = (s1.rot.Transform(c.pos1) + s1.pos - s2.pos).Skew() * (s1.rot * c.rot1.Conjugate()).ToMatrix();
      mat_to_vars(m, c.body2*6 + 3, var, pos_dof);
      var += __builtin_popcount(pos_dof);
    }
    if (c.lock & Constraint::DOF::ROT) {
    }
  }

  context.equations.Resize(nvars, nvars + 1);
  context.equations.FillIdentity();
  // TODO: Fill in the equations.

  bool ok = context.equations.SolveLinearSystem();
  ++(ok ? scene.force_resolution_success : scene.force_resolution_failed);

  for (size_t i = 0; i < fv.n; ++i) {
    for (size_t j = 0; j < nvars; ++j)
      fv[i][nvars] += fv[i][j] * context.equations[j][nvars];
  }

  context.effective_forces.resize(scene.bodies.size());
  for (size_t i = 0; i < scene.bodies.size(); ++i) {
    context.effective_forces[i].force.FromArray(fv[i*6] + nvars, fv.Stride());
    context.effective_forces[i].torque.FromArray(fv[i*6+3] + nvars, fv.Stride());
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
  Context context;
  context.var_idx.resize(constraints.size() + 1);
  for (size_t i = 0; i < constraints.size(); ++i) {
    size_t n = __builtin_popcount(constraints[i].lock);
    context.var_idx[i + 1] = context.var_idx[i] + n;
  }
  context.external_forces.resize(bodies.size());
  StateVector state_vec(bodies.size());
  for (size_t i = 0; i < bodies.size(); ++i) {
    Body& body = bodies[i];
    for (const auto& f: body.forces) {
      context.external_forces[i].force += f.second;
      context.external_forces[i].torque += f.first.Cross(f.second);
    }
    state_vec[i].FromBody(body);
  }
  auto f = [&](const StateVector& y, StateVector& yp) {
    ResolveForces(*this, y, context);
    for (size_t i = 0; i < bodies.size(); ++i) {
      const Body& body = bodies[i];
      const BodyState& s = y[i];
      BodyState& p = yp[i];
      p.pos = s.momentum / body.mass;
      dvec3 av = body.inv_inertia * (s.rot.ToMatrix().Transposed() * s.ang);
      p.rot = s.rot * fquat(0, av.x, av.y, av.z) * .5;
      p.momentum = context.effective_forces[i].force;
      p.ang = context.effective_forces[i].torque;
    }
  };
  const int steps = 100;
  for (int i = 0; i < steps; ++i) {
    RungeKutta4(state_vec, dt / steps, f);
  }
  for (size_t i = 0; i < bodies.size(); ++i) {
    Body& body = bodies[i];
    state_vec[i].ToBody(body);
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
