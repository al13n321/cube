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

BodyEdit MakeCylinder(double r, double h) {
  BodyEdit b;
  b.mass = M_PI*r*r*h;
  b.com = fvec3(0, 0, 0);
  b.inertia = b.mass/12 * dmat3::Diag(3*r*r + h*h, 6*r*r, 3*r*r + h*h);

  using Vertex = BodyEdit::Vertex;
  const int sides = 100;
  for (int i = 0; i < sides; ++i) {
    float a1 = M_PI * 2 / sides * (i-.5);
    float a2 = M_PI * 2 / sides * (i+.5);
    fvec3 n1(sin(a1), 0, cos(a1));
    fvec3 n2(sin(a2), 0, cos(a2));
    fvec3 u0(0,  h/2, 0);
    fvec3 d0(0, -h/2, 0);
    fvec3 u1 = n1*r + u0;
    fvec3 u2 = n2*r + u0;
    fvec3 d1 = n1*r + d0;
    fvec3 d2 = n2*r + d0;
    size_t sz0 = b.vertices.size();
    b.vertices.push_back(Vertex(d1, n1));
    b.vertices.push_back(Vertex(d2, n2));
    b.vertices.push_back(Vertex(u2, n2));
    b.vertices.push_back(Vertex(u2, n2));
    b.vertices.push_back(Vertex(u1, n1));
    b.vertices.push_back(Vertex(d1, n1));
    b.vertices.push_back(Vertex(u1, {0, 1, 0}));
    b.vertices.push_back(Vertex(u2, {0, 1, 0}));
    b.vertices.push_back(Vertex(u0, {0, 1, 0}));
    b.vertices.push_back(Vertex(d2, {0, -1, 0}));
    b.vertices.push_back(Vertex(d1, {0, -1, 0}));
    b.vertices.push_back(Vertex(d0, {0, -1, 0}));
    if (!i) {
      // A red stripe to make rotation visible.
      for (size_t j = sz0; j < b.vertices.size(); ++j)
        b.vertices[j].color = fvec3(1, 0, 0);
    }
  }
  return b;
}
BodyEdit MakeTube(double in_r, double out_r, double h) {
  BodyEdit b;
  b.mass = M_PI*(out_r*out_r - in_r*in_r)*h;
  b.com = fvec3(0, 0, 0);
  double t = in_r*in_r + out_r*out_r;
  b.inertia = b.mass/12 * dmat3::Diag(3*t + h*h, 6*t, 3*t + h*h);
  throw NotImplementedException();
}
