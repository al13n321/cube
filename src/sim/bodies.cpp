#include "sim/scene.h"
#include "util/exceptions.h"

BodyEdit& BodyEdit::AddColor(fvec3 color) {
  for (Vertex& v: vertices)
    v.color += color;
  return *this;
}

BodyEdit& BodyEdit::MultiplyMass(double factor) {
  mass *= factor;
  inertia *= factor;
  return *this;
}

// Inertia tensor around origin of a unit mass in point p.
static dmat3 PointInertia(dvec3 p) {
  dmat3 m(0, -p.z, p.y, p.z, 0, -p.x, -p.y, p.x, 0);
  return -m*m;
}

BodyEdit& BodyEdit::Merge(const BodyEdit& b) {
  vertices.insert(vertices.end(), b.vertices.begin(), b.vertices.end());
  dvec3 ncom = (com * mass + b.com * b.mass) / (mass + b.mass);
  // Inertia around origin is inertia around c.o.m. + inertia of c.o.m. around origin.
  // Only works for center of mass, not for any point.
  inertia += b.inertia + PointInertia(com - ncom)*mass + PointInertia(b.com - ncom)*b.mass;
  com = ncom;
  mass += b.mass;
  return *this;
}

BodyEdit& BodyEdit::Translate(dvec3 d) {
  for (Vertex& v: vertices)
    v.pos += fvec3(d);
  com += d;
  return *this;
}

BodyEdit& BodyEdit::Rotate(dquat q) {
  for (Vertex& v: vertices) {
    v.pos = q.Transform(v.pos);
    v.normal = q.Transform(v.normal);
  }
  com = q.Transform(com);
  dmat3 m = q.ToMatrix();
  inertia = m*inertia*m.Transposed();
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

BodyEdit MakeTube(double in_r, double out_r, double h) {
  BodyEdit b;
  b.mass = M_PI*(out_r*out_r - in_r*in_r)*h;
  b.com = fvec3(0, 0, 0);
  double t = in_r*in_r + out_r*out_r;
  b.inertia = b.mass/12 * dmat3::Diag(3*t + h*h, 6*t, 3*t + h*h);

  using Vertex = BodyEdit::Vertex;
  const int sides = 100;
  for (int i = 0; i < sides; ++i) {
    float a1 = M_PI * 2 / sides * (i-.5);
    float a2 = M_PI * 2 / sides * (i+.5);
    fvec3 n1(sin(a1), 0, cos(a1));
    fvec3 n2(sin(a2), 0, cos(a2));
    fvec3 nu(0, 1, 0);
    fvec3 u(0,  h/2, 0);
    fvec3 o1 = n1*out_r;
    fvec3 o2 = n2*out_r;
    size_t sz0 = b.vertices.size();
    // Outer face.
    b.vertices.push_back(Vertex(o1 - u, n1));
    b.vertices.push_back(Vertex(o2 - u, n2));
    b.vertices.push_back(Vertex(o2 + u, n2));
    b.vertices.push_back(Vertex(o2 + u, n2));
    b.vertices.push_back(Vertex(o1 + u, n1));
    b.vertices.push_back(Vertex(o1 - u, n1));
    if (in_r > 0) { // tube
      fvec3 i1 = n1*in_r;
      fvec3 i2 = n2*in_r;
      // Inner face.
      b.vertices.push_back(Vertex(i1 + u, n1));
      b.vertices.push_back(Vertex(i2 + u, n2));
      b.vertices.push_back(Vertex(i2 - u, n2));
      b.vertices.push_back(Vertex(i2 - u, n2));
      b.vertices.push_back(Vertex(i1 - u, n1));
      b.vertices.push_back(Vertex(i1 + u, n1));
      // Top face.
      b.vertices.push_back(Vertex(o1 + u, nu));
      b.vertices.push_back(Vertex(o2 + u, nu));
      b.vertices.push_back(Vertex(i2 + u, nu));
      b.vertices.push_back(Vertex(i2 + u, nu));
      b.vertices.push_back(Vertex(i1 + u, nu));
      b.vertices.push_back(Vertex(o1 + u, nu));
      // Bottom face.
      b.vertices.push_back(Vertex(o2 - u, -nu));
      b.vertices.push_back(Vertex(o1 - u, -nu));
      b.vertices.push_back(Vertex(i1 - u, -nu));
      b.vertices.push_back(Vertex(i1 - u, -nu));
      b.vertices.push_back(Vertex(i2 - u, -nu));
      b.vertices.push_back(Vertex(o2 - u, -nu));      
    } else { // cylinder
      // Top face.
      b.vertices.push_back(Vertex(o1 + u, nu));
      b.vertices.push_back(Vertex(o2 + u, nu));
      b.vertices.push_back(Vertex(u, nu));
      // Bottom face.
      b.vertices.push_back(Vertex(o2 - u, -nu));
      b.vertices.push_back(Vertex(o1 - u, -nu));
      b.vertices.push_back(Vertex(-u, -nu));
    }
    if (!i) {
      // A red stripe to make rotation visible.
      for (size_t j = sz0; j < b.vertices.size(); ++j)
        b.vertices[j].color = fvec3(1, 0, 0);
    }
  }
  return b;
}

BodyEdit MakeTHandle(double rh, double lh, double rv, double lv) {
  BodyEdit h = MakeCylinder(rh, lh);
  h.Rotate(dquat(M_PI/2, dvec3(0, 0, 1)));
  h.Translate(dvec3(0, lv/2, 0));
  h.Merge(MakeCylinder(rv, lv));
  return h;
}
