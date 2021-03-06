#include "sim/scene.h"
#include "util/linear.h"
#include "util/print.h"
#include <valarray>
#include <cassert>
#include <iostream>
using namespace std;

namespace {

static const Body fixed_body = Body(-1);

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

  static BodyState Zero() {
    BodyState s;
    memset(&s, 0, sizeof(s));
    s.rot.a = 1;
    return s;
  }
};

static const BodyState fixed_body_state = BodyState::Zero();

struct BodyForce {
  dvec3 force = {0, 0, 0};
  dvec3 torque = {0, 0, 0};
};
ostream& operator<<(ostream& o, const BodyForce& f) __attribute__ ((unused));
ostream& operator<<(ostream& o, const BodyForce& f) {
  return o << "(F:" << f.force << ",tau:" << f.torque << ")";
}

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

  // *this = s + h * v;  s is allowed to point to *this
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
  // Force and torque on each body represented as linear combination of variables.
  // [(v+1)*(b*2+f) + i] is the contribution of variable i to force/torque f on body b,
  // v is number of variables, b is body idx, f is 0 for linear force, 1 for torque,
  // i is variable index, v for free coefficient (with opposite sign).
  vector<dvec3> force_from_vars;
  // Linear equation system. Vars - forces/torques from constraints,
  // rows - constraints (second derivative), last column - "b" as in Ax=b.
  DMatrix equations;
};

// According to [1], this method has only second order accuracy for rotations.
// Still, it seems to perform somewhat better than MidpointMethod() in my experiments.
// [1] http://euclid.ucsd.edu/~sbuss/ResearchWeb/accuraterotation/paper.pdf
void RungeKutta4(StateVector& y, double h, function<void(const StateVector& y, StateVector& yp)> f)
  __attribute__((unused));
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
}

void MidpointMethod(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f)
  __attribute__((unused));
void MidpointMethod(valarray<double>& y, double h, function<void(const valarray<double>& y, valarray<double>& yp)> f) {
  valarray<double> k1(y.size()), k2(y.size());
  f(y, k1);
  f(y + h/2*k1, k2);
  y += h*k2;
}//*/

// If you want to see the difference between first-order and second-order integration,
// try using Euler() instead of RungeKutta4() (also fewer substeps).
// Torque-free precession of this single rotating box degrades in a few seconds with Euler():
// scene.AddBody(MakeBox(dvec3(.2, .1, .3)).MultiplyMass(2700))->ang = dvec3(0,-1.24991,-0.758193);
void Euler(StateVector& y, double h, function<void(const StateVector& y, StateVector& yp)> f)
  __attribute__((unused));
void Euler(StateVector& y, double h, function<void(const StateVector& y, StateVector& yp)> f) {
  StateVector k(y.size());
  f(y, k);
  y.AddMul(y, h, k);
}

// Fills context.effective_forces.
void ResolveForces(Scene& scene, const StateVector& state, Context& context) {
  size_t nvars = context.var_idx.back();
  auto& fv = context.force_from_vars;
  fv.assign(scene.bodies.size() * 2 * (nvars + 1), dvec3(0, 0, 0));

  for (size_t i = 0; i < scene.bodies.size(); ++i) {
    fv[(i*2 + 0)*(nvars+1) + nvars] = -context.external_forces[i].force;
    fv[(i*2 + 1)*(nvars+1) + nvars] = -context.external_forces[i].torque;
  }

  auto get_mask = [](Constraint::dof_t dofs) {
    return (uint8_t)((dofs / Constraint::DOF::PX) | (dofs / Constraint::DOF::RX));
  };
  auto mat_to_vars = [&](const dmat3& m, size_t i, uint8_t msk) {
    for (size_t j = 0; j < 3; ++j) {
      if (!(msk & (1 << j)))
        continue;
      fv[i++] += m.Column(j);
    }
  };
  for (size_t i = 0; i < scene.constraints.size(); ++i) {
    const Constraint& c = scene.constraints[i];
    const BodyState& s1 = c.body1 == -1 ? fixed_body_state : state[c.body1];
    const BodyState& s2 = state[c.body2];
    size_t var = context.var_idx[i];
    const dmat3 c2w = (s1.rot * c.rot1.Conjugate()).ToMatrix();
    if (auto pos_dof = get_mask(c.lock & Constraint::DOF::POS)) {
      if (c.body1 != -1)
        mat_to_vars(-c2w, c.body1*2*(nvars+1) + var, pos_dof);
      mat_to_vars(c2w, c.body2*2*(nvars+1) + var, pos_dof);
      // Body torque depends on constraint force too (not only on constraint torque).
      dmat3 m = -s1.rot.ToMatrix() * c.pos1.Skew() * c.rot1.Conjugate().ToMatrix();
      if (c.body1 != -1)
        mat_to_vars(m, (c.body1*2 + 1)*(nvars+1) + var, pos_dof);
      m = (s1.rot.Transform(c.pos1) + s1.pos - s2.pos).Skew() * (s1.rot * c.rot1.Conjugate()).ToMatrix();
      mat_to_vars(m, (c.body2*2 + 1)*(nvars+1) + var, pos_dof);
      var += __builtin_popcount(pos_dof);
    }
    if (auto rot_dof = get_mask(c.lock & Constraint::DOF::ROT)) {
      if (c.body1 != -1)
        mat_to_vars(-c2w, (c.body1*2+1)*(nvars+1) + var, rot_dof);
      mat_to_vars(c2w, (c.body2*2+1)*(nvars+1) + var, rot_dof);
      var += __builtin_popcount(rot_dof);
    }
  }

  context.equations.Resize(nvars, nvars + 1);
  context.equations.Fill(0);

  auto mat_to_equations = [&](const dmat3& m, size_t ei, size_t fi, Constraint::dof_t msk) {
    for (size_t j = 0; j <= nvars; ++j) {
      dvec3 v = m * fv[fi + j];
      v.AddToArrayMasked(context.equations[ei] + j, msk, context.equations.Stride());
    }
  };
  for (size_t i = 0; i < scene.constraints.size(); ++i) {
    const Constraint& c = scene.constraints[i];
    const Body& b1 = c.body1 == -1 ? fixed_body : scene.bodies[c.body1];
    const Body& b2 = scene.bodies[c.body2];
    const BodyState& s1 = c.body1 == -1 ? fixed_body_state : state[c.body1];
    const BodyState& s2 = state[c.body2];
    size_t eq = context.var_idx[i];
    dvec3 av1 = b1.inv_inertia * s1.rot.Untransform(s1.ang);
    dvec3 av2 = b2.inv_inertia * s2.rot.Untransform(s2.ang);
    if (auto pos_dof = get_mask(c.lock & Constraint::DOF::POS)) {
      // Second derivative of (2): add + cf1*force1 + cf2*force2 + ct1*torque1 + ct2*torque2.
      dvec3 add = c.rot1.Transform((b1.inv_inertia*av1.Cross(s1.rot.Untransform(s1.ang)))
                                   .Cross(s1.rot.Untransform(s2.rot.Transform(c.pos2)+s2.pos-s1.pos)) + // precession 1
                                   av1.Cross(av1.Cross(s1.rot.Untransform(s2.rot.Transform(c.pos2)+s2.pos-s1.pos)) + // centripetal 1
                                             -2.*s1.rot.Untransform(s2.rot.Transform(av2.Cross(c.pos2)) +
                                                                    s2.momentum*b2.inv_mass - s1.momentum*b1.inv_mass)) + // Coriolis
                                   s1.rot.Untransform(s2.rot.Transform(av2.Cross(av2.Cross(c.pos2)) + // centripetal 2
                                                                       c.pos2.Cross(b2.inv_inertia*av2.Cross(s2.rot.Untransform(s2.ang))))) // precession 2
                                   );
      (-add).AddToArrayMasked(context.equations[eq] + nvars, pos_dof, context.equations.Stride());
      dmat3 cf2 = (c.rot1 * s1.rot.Conjugate()).ToMatrix();
      dmat3 cf1 = cf2 * -b1.inv_mass;
      cf2 *= b2.inv_mass;
      if (c.body1 != -1)
        mat_to_equations(cf1, eq, c.body1*2*(nvars+1), pos_dof);
      mat_to_equations(cf2, eq, c.body2*2*(nvars+1), pos_dof);
      dmat3 ct1 = c.rot1.ToMatrix() * (s1.rot.Untransform(s2.rot.Transform(c.pos2)+s2.pos-s1.pos)).Skew() * b1.inv_inertia * s1.rot.Conjugate().ToMatrix();
      dmat3 ct2 = -(c.rot1*s1.rot.Conjugate()*s2.rot).ToMatrix() * c.pos2.Skew() * b2.inv_inertia * s2.rot.Conjugate().ToMatrix();
      if (c.body1 != -1)
        mat_to_equations(ct1, eq, (c.body1*2+1)*(nvars+1), pos_dof);
      mat_to_equations(ct2, eq, (c.body2*2+1)*(nvars+1), pos_dof);
      eq += __builtin_popcount(pos_dof);
    }
    if (auto rot_dof = get_mask(c.lock & Constraint::DOF::ROT)) {
      // Derivative of (1): add + ct1*torque1 + ct2*torque2.
      dvec3 add = c.rot1.Transform(-av1.Cross(s1.rot.Untransform(s2.rot.Transform(av2)))+
                                   -s1.rot.Untransform(s2.rot.Transform(b2.inv_inertia*av2.Cross(s2.rot.Untransform(s2.ang))))+
                                   b1.inv_inertia*av1.Cross(s1.rot.Untransform(s1.ang)));
      (-add).AddToArrayMasked(context.equations[eq] + nvars, rot_dof, context.equations.Stride());
      dmat3 ct1 = -c.rot1.ToMatrix() * b1.inv_inertia * s1.rot.Conjugate().ToMatrix();
      dmat3 ct2 = (c.rot1 * s1.rot.Conjugate() * s2.rot).ToMatrix() * b2.inv_inertia * s2.rot.Conjugate().ToMatrix();
      if (c.body1 != -1)
        mat_to_equations(ct1, eq, (c.body1*2+1)*(nvars+1), rot_dof);
      mat_to_equations(ct2, eq, (c.body2*2+1)*(nvars+1), rot_dof);
      eq += __builtin_popcount(rot_dof);
    }
  }

  bool ok = context.equations.SolveLinearSystem();
  ++(ok ? scene.force_resolution_success : scene.force_resolution_failed);

  for (size_t i = 0; i < fv.size(); i += nvars+1) {
    for (size_t j = 0; j < nvars; ++j)
      fv[i + nvars] -= fv[i + j] * context.equations[j][nvars];
  }

  context.effective_forces.resize(scene.bodies.size());
  for (size_t i = 0; i < scene.bodies.size(); ++i) {
    context.effective_forces[i].force = -fv[i*2*(nvars+1) + nvars];
    context.effective_forces[i].torque = -fv[(i*2+1)*(nvars+1) + nvars];
  }
}

} // namespace {

// Prevent errors from accumulating by coercing the bodies into meeting all constraints,
// together with their first derivative, in a physically incorrect way. Done after a normal update step.
void Scene::EnforceConstraints() {
  for (const Constraint& c: constraints) {
    const Body& b1 = c.body1 == -1 ? fixed_body : bodies[c.body1];
    Body& b2 = bodies[c.body2];

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
      leaked_rotation += sqrt(l);
    }

    // Translation.
    {
      dvec3 p = c.rot1.Transform(b1.rot.Untransform(b2.rot.Transform(c.pos2) + b2.pos - b1.pos) - c.pos1); // (2)
      double l = clear_vec(p, Constraint::DOF::POS);
      b2.pos = b1.rot.Transform(c.rot1.Untransform(p) + c.pos1) + b1.pos - b2.rot.Transform(c.pos2);
      leaked_translation += sqrt(l);
    }

    // Angular velocity.
    {
      dvec3 w = c.rot1.Transform(b1.rot.Untransform(b2.rot.Transform(b2.inv_inertia * b2.rot.Untransform(b2.ang))) -
                                 b1.inv_inertia * b1.rot.Untransform(b1.ang)); // (1)
      double l = clear_vec(w, Constraint::DOF::ROT);
      b2.ang = b2.rot.Transform(b2.inv_inertia.Inverse() *
                                b2.rot.Untransform(b1.rot.Transform(c.rot1.Untransform(w) +
                                                                    b1.inv_inertia * b1.rot.Untransform(b1.ang))));
      leaked_angular_velocity += sqrt(l);
    }

    // Linear velocity.
    {
      dvec3 av1 = b1.inv_inertia * b1.rot.Untransform(b1.ang);
      dvec3 av2 = b2.inv_inertia * b2.rot.Untransform(b2.ang);
      dvec3 v0 =
        -av1.Cross(b1.rot.Untransform(b2.rot.Transform(c.pos2)+b2.pos-b1.pos)) +
        b1.rot.Untransform(b2.rot.Transform(av2.Cross(c.pos2)) - b1.momentum*b1.inv_mass);
      // Derivative of (2).
      dvec3 v = c.rot1.Transform(v0 + b1.rot.Untransform(b2.momentum*b2.inv_mass));
      double l = clear_vec(v, Constraint::DOF::POS);
      b2.momentum = b1.rot.Transform((c.rot1.Untransform(v) - v0)) / b2.inv_mass;
      leaked_velocity += sqrt(l);
    }
  }
}

double Scene::GetEnergy() const {
  double r = 0;
  for (const Body& b: bodies) {
    dvec3 ang_in_body = b.rot.Untransform(b.ang);
    r += .5 * (b.momentum.LengthSquare() * b.inv_mass + ang_in_body.Dot(b.inv_inertia * ang_in_body));
    r -= b.pos.Dot(gravity) / b.inv_mass;
  }
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
      context.external_forces[i].torque += (f.first - body.pos).Cross(f.second);
    }
    context.external_forces[i].force += gravity / body.inv_mass;
    state_vec[i].FromBody(body);
  }
  auto f = [&](const StateVector& y, StateVector& yp) {
    ResolveForces(*this, y, context);
    for (size_t i = 0; i < bodies.size(); ++i) {
      const Body& body = bodies[i];
      const BodyState& s = y[i];
      BodyState& p = yp[i];
      p.pos = s.momentum * body.inv_mass;
      dvec3 av = body.inv_inertia * (s.rot.ToMatrix().Transposed() * s.ang);
      p.rot = s.rot * fquat(0, av.x, av.y, av.z) * .5;
      p.momentum = context.effective_forces[i].force;
      p.ang = context.effective_forces[i].torque;
    }
  };
  const int steps = 100;
  for (int i = 0; i < steps; ++i) {
    RungeKutta4(state_vec, dt / steps, f);
    //Euler(state_vec, dt / steps, f);
  }
  for (size_t i = 0; i < bodies.size(); ++i) {
    Body& body = bodies[i];
    state_vec[i].ToBody(body);
    body.rot.NormalizeMe();
  }
}

Constraint* Scene::AddConstraint(int body1, int body2, dvec3 pos2, dquat rot2, Constraint::dof_t lock) {
  assert(body1 >= -1);
  assert(body1 < (int)bodies.size());
  assert(body2 >= 0);
  assert(body2 < (int)bodies.size());

  const Body& b1 = body1 == -1 ? fixed_body : bodies[body1];
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
