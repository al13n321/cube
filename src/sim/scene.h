#pragma once
#include "util/vec.h"
#include "util/quat.h"
#include "gl-util/gl-common.h"
#include "gl-util/vertex-array.h"
#include "gl-util/shader.h"
#include <vector>
#include <list>
#include <deque>

// The implementation of functions declared here is somewhat arbitrarily split between render.cpp and phys.cpp.

// Mutable body that is not put in a scene yet.
// These things can be transformed and combined;
// you can assemble a compound body from primitives.
// After you're done, you can turn this into a `Body` and put in a scene.
class BodyEdit { 
 public:
  struct Vertex { // directly copied to opengl vertex buffer object
    fvec3 pos;
    fvec3 normal;
    fvec3 color = {.9, .9, .9};

    Vertex() = default;
    Vertex(fvec3 pos, fvec3 normal): pos(pos), normal(normal) {}
  };

  std::vector<Vertex> vertices;
  double mass;
  dvec3 com; // center of mass
  dmat3 inertia; // around center of mass

  BodyEdit& AddColor(fvec3 color);

  // Create union of `*this` and `rhs` and put it into `*this`.
  BodyEdit& Merge(const BodyEdit& rhs);
  BodyEdit& MultiplyMass(double factor);
  BodyEdit& Translate(dvec3 d);
  BodyEdit& Rotate(dquat q);
  BodyEdit& Scale(double factor);
};

class Mesh {
 public:
  fvec3 tint = fvec3(0, 0, 0);
  std::unique_ptr<GL::VertexArray> vao;
};

class Body {
 public:
  // Identifier.
  int idx; // index in list of all entities

  // Physical properties. Mass is stored as reciprocal to allow infinite mass.
  double inv_mass = 0; // 1/mass
  dmat3 inv_inertia = dmat3::Zero(); // inverse of inertia tensor

  // Physical state.
  dvec3 pos = dvec3(0, 0, 0); // center of mass in world space
  dquat rot = dquat(1, 0, 0, 0); // rotation; object->world space
  dvec3 momentum = dvec3(0, 0, 0); // momentum of c.o.m. in world space
  dvec3 ang = dvec3(0, 0, 0); // angular momentum in world space

  // Forces applied to this body.
  // `first` is point of application (in world space), `second` is force vector.
  std::list<std::pair<dvec3, dvec3>> forces;

  // How to render it.
  Mesh mesh;

  Body(int idx): idx(idx) {}
};

// A generic constraint, similar to e.g. btGeneric6DofConstraint in Bullet Physics.
// It's probably better to use the bigger/more-stationary body as body1 and the more movable as body2.
// (I don't know how much better yet. body2 is slightly moved in non-physical ways to compensate for numerical errors.)
struct Constraint {
  using dof_t = uint8_t;

  enum DOF: dof_t {
    PX = 1,
    PY = 2,
    PZ = 4,
    RX = 8,
    RY = 16,
    RZ = 32,

    POS = PX | PY | PZ,
    ROT = RX | RY | RZ,
  };

  int body1; // -1 for world
  int body2; // can't be -1

  // constraint point in body space
  dvec3 pos1;
  dvec3 pos2;
  // body -> constraint space
  dquat rot1;
  dquat rot2;

  // Which degrees of freedom to lock.
  // E.g. POS is ball socket, POS | RX | RY is hinge constraint.
  dof_t lock;
};

struct Camera {
 public:
  fvec3 pos = fvec3(0, 0, 0);
  float yaw = 0; // counterclockwise from -z axis
  float pitch = 0; // upwards
  float fov = M_PI/2;
  float aspect_ratio = 1;

  void LookAt(fvec3 p);
  fmat4 ViewProjection() const;
};

class Scene {
 public:
  Scene();

  Body* AddBody();
  Body* AddBody(BodyEdit edit); // puts c.o.m. at origin
  // pos1 and rot1 are calculated from current positions and orientations of the two bodies.
  Constraint* AddConstraint(int body1, int body2, dvec3 pos2, dquat rot2, Constraint::dof_t lock);

  // If constraints are not satisfied (by either position, rotation, velocity or angular velocity),
  // updates them (in some physically incorrect way) to satisfy constraints.
  // Call if you changed velocities manually or if you called AddConstraint() for moving bodies.
  void EnforceConstraints();

  void Render();
  void PhysicsStep(double dt);

  double GetEnergy() const;

  std::deque<Body> bodies;
  std::deque<Constraint> constraints;
  dvec3 gravity = dvec3(0, 0, 0);

  Camera camera;
  fvec3 light_vec = fvec3(-3, 2, 1).Normalized(); // direction from which the light is coming

  // Stats.

  // From constraints.
  double leaked_translation = 0;
  double leaked_rotation = 0;
  double leaked_velocity = 0;
  double leaked_angular_velocity = 0;

  size_t force_resolution_success = 0;
  size_t force_resolution_failed = 0;

 private:
  GL::Shader shader_;
};

// All of unit density. Use `MultiplyMass()` to set density afterwards.
BodyEdit MakeBox(dvec3 size);
BodyEdit MakeTube(double in_r, double out_r, double h); // Difference of two cylinders with common axis.
inline BodyEdit MakeCylinder(double r, double h) {
  return MakeTube(0, r, h);
}
BodyEdit MakeTHandle(double rh=.015, double lh=.15, double rv=.0075, double lv=.09);
