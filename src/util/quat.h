#pragma once

#include "vec.h"
#include "mat.h"

// Quaternion.
struct fquat {
  float a{1}, b{0}, c{0}, d{0};

  fquat() {}
  fquat(float a, float b, float c, float d): a(a), b(b), c(c), d(d) {}

  // Angle in radians. Angle is measured clockwise
  // when viewed with view direction equal to axis.
  fquat(float angle, fvec3 axis) {
    float sn = sinf(angle * .5f);
    float cs = cosf(angle * .5f);
    a = cs;
    axis.NormalizeMe();
    axis *= sn;
    b = axis.x;
    c = axis.y;
    d = axis.z;
  }

  fquat operator+(const fquat &q) const {
    return fquat(a+q.a, b+q.b, c+q.c, d+q.d);
  }
  fquat operator*(const fquat &q) const {
    return fquat(
      a*q.a - b*q.b - c*q.c - d*q.d,
      a*q.b + b*q.a + c*q.d - d*q.c,
      a*q.c + c*q.a + d*q.b - b*q.d,
      a*q.d + d*q.a + b*q.c - c*q.b);
  }
  fquat& operator*=(const fquat &q) {
    return *this = *this * q;
  }
  fquat operator*(float s) const {
    return fquat(a*s, b*s, c*s, d*s);
  }
  fquat operator/(float s) const {
    return fquat(a/s, b/s, c/s, d/s);
  }

  void NormalizeMe() {
    float z = sqrtf(a*a + b*b + c*c + d*d);
    a /= z;
    b /= z;
    c /= z;
    d /= z;
  }
  fquat Normalized() const {
    fquat res = *this;
    res.NormalizeMe();
    return res;
  }

  fmat4 ToMatrix() const {
    // Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/
    // (a bit modified)
    return fmat4(
       a, -d,  c, -b,
       d,  a, -b, -c,
      -c,  b,  a, -d,
       b,  c,  d,  a
    ) * fmat4(
       a, -d,  c,  b,
       d,  a, -b,  c,
      -c,  b,  a,  d,
      -b, -c, -d,  a
    );
  }

  fquat Conjugate() const {
    return fquat(a, -b, -c, -d);
  }

  fquat Inverse() const {
    return Conjugate() / (a*a + b*b + c*c + d*d);
  }

  fvec3 Transform(fvec3 v) const {
    fquat q = *this * fquat(0, v.x, v.y, v.z) * this->Inverse();
    return fvec3(q.b, q.c, q.d);
  }
};
