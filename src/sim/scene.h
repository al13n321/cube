#pragma once
#include "util/vec.h"
#include "util/quat.h"
#include "gl-util/gl-common.h"
#include "gl-util/vertex-array.h"
#include "gl-util/shader.h"
#include <vector>
#include <list>

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
  float mass;
  fvec3 com; // center of mass
  fmat3 inertia; // around origin

  BodyEdit& SetColor(fvec3 color);

  // Create union of `*this` and `rhs` and put it into `*this`.
  BodyEdit& Merge(const BodyEdit& rhs);
  BodyEdit& MultiplyMass(float factor);
  BodyEdit& Translate(fvec3 d);
  BodyEdit& Rotate(fquat q);
};

class Model {
 public:
  fvec3 tint = fvec3(0, 0, 0);
  std::unique_ptr<GL::VertexArray> vao;
};

class Body {
 public:
  // Identifier.
  size_t idx; // index in list of all entities

  // Physical properties.
  float mass = 0;
  fmat3 inv_inertia = fmat3::Zero(); // inverse of inertia tensor

  // Physical state.
  fvec3 pos = fvec3(0, 0, 0); // center of mass in world space
  fquat rot = fquat(1, 0, 0, 0); // rotation; object->world space
  fvec3 momentum = fvec3(0, 0, 0); // momentum of c.o.m. in world space
  fvec3 ang = fvec3(0, 0, 0); // angular momentum in world space

  // Forces applied to this body.
  // `first` is point of application (in world space), `second` is force vector.
  std::list<std::pair<fvec3, fvec3>> forces;

  // How to render it.
  Model model;

  Body(size_t idx): idx(idx) {}
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
  std::list<Body> bodies;
  Camera camera;
  fvec3 light_vec = fvec3(-3, 2, 1).Normalized(); // direction from which the light is coming

  Scene();

  Body* AddBody();
  Body* AddBody(BodyEdit edit); // puts c.o.m. at origin

  void Render();
  void PhysicsStep(double dt);

 private:
  GL::Shader shader_;
};

// All of unit density. Use `MultiplyMass()` to set density afterwards.
BodyEdit MakeBox(fvec3 size);
BodyEdit MakeTorus(float cyl_rad, float mid_rad);
BodyEdit MakeCylinder(float r, float h);