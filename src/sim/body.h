#pragma once
#include "util/vec.h"
#include "util/quat.h"

class Body {
 public:
  // Identifier.
  size_t idx; // index in list of all entities

  // Physical properties.
  float mass;
  fmat3 inertia; // inertia tensor

  // Physical state.
  fvec3 pos; // center of mass in world space
  fquat rot; // rotation; object->world space
  fvec3 momentum; // momentum of c.o.m. in world space
  fvec3 ang; // angular momentum in world space

  // How to render it.
  fvec3 color;
  vector<array<fvec3, 3>> triangles;
};
