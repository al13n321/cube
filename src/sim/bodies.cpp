#include "sim/scene.h"
#include "util/exceptions.h"

BodyEdit& BodyEdit::SetColor(fvec3 color) {
  for (Vertex& v: vertices)
    v.color = color;
  return *this;
}

BodyEdit& BodyEdit::MultiplyMass(double factor) {
  mass *= factor;
  inertia *= factor;
  return *this;
}

BodyEdit& BodyEdit::Merge(const BodyEdit& b) {
  throw NotImplementedException();
  return *this;
}

BodyEdit& BodyEdit::Translate(dvec3 d) {
  if (d.Length() > 1e-12)
    throw NotImplementedException();
  return *this;
}

BodyEdit& BodyEdit::Rotate(dquat q) {
  throw NotImplementedException();
  return *this;
}

BodyEdit MakeBox(dvec3 s) {
  BodyEdit b;
  b.mass = s.x * s.y * s.z;
  b.com = fvec3(0, 0, 0);
  b.inertia = b.mass/12. * dmat3::Diag(s.y*s.y + s.z*s.z,
                                       s.x*s.x + s.z*s.z,
                                       s.x*s.x + s.y*s.y);
  fvec3 h = s/2;
  b.vertices.reserve(6 * 2 * 3);
  using Vertex = BodyEdit::Vertex;
  auto face=[&](fvec3 v0, fvec3 v1, fvec3 v2, fvec3 v3, fvec3 n) {
    b.vertices.push_back(Vertex(v0, n));
    b.vertices.push_back(Vertex(v1, n));
    b.vertices.push_back(Vertex(v2, n));
    b.vertices.push_back(Vertex(v2, n));
    b.vertices.push_back(Vertex(v3, n));
    b.vertices.push_back(Vertex(v0, n));
  };
  face({ h.x,  h.y, h.z}, { h.x,  h.y, -h.z}, {-h.x,  h.y, -h.z}, {-h.x,  h.y,  h.z}, { 0,  1,  0}); // top
  face({ h.x, -h.y, h.z}, {-h.x, -h.y,  h.z}, {-h.x, -h.y, -h.z}, { h.x, -h.y, -h.z}, { 0, -1,  0}); // bottom
  face({ h.x,  h.y, h.z}, { h.x, -h.y,  h.z}, { h.x, -h.y, -h.z}, { h.x,  h.y, -h.z}, { 1,  0,  0}); // right
  face({-h.x,  h.y, h.z}, {-h.x,  h.y, -h.z}, {-h.x, -h.y, -h.z}, {-h.x, -h.y,  h.z}, {-1,  0,  0}); // left
  face({ h.x,  h.y, h.z}, {-h.x,  h.y,  h.z}, {-h.x, -h.y,  h.z}, { h.x, -h.y,  h.z}, { 0,  0,  1}); // front
  face({ h.x,  h.y,-h.z}, { h.x, -h.y, -h.z}, {-h.x, -h.y, -h.z}, {-h.x,  h.y, -h.z}, { 0,  0, -1}); // back
  return b;
}

BodyEdit MakeTorus(double cyl_rad, double mid_rad) {
  throw NotImplementedException();
}
BodyEdit MakeCylinder(double r, double h) {
  throw NotImplementedException();
}
