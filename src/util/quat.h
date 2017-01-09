#pragma once

#include "vec.h"
#include "mat.h"
#include <ostream>

// Quaternion.
template<typename T>
struct tquat {
  T a{1}, b{0}, c{0}, d{0};

  tquat() {}
  tquat(T a, T b, T c, T d): a(a), b(b), c(c), d(d) {}
  template<typename T2>
  tquat(const tquat<T2>& q): a((T)q.a), b((T)q.b), c((T)q.c), d((T)q.d) {}

  // Angle in radians. Angle is measured clockwise
  // when viewed with view direction equal to axis.
  tquat(T angle, tvec3<T> axis) {
    T sn = std::sin(angle * .5f);
    T cs = std::cos(angle * .5f);
    a = cs;
    axis.NormalizeMe();
    axis *= sn;
    b = axis.x;
    c = axis.y;
    d = axis.z;
  }

  tquat operator+(const tquat &q) const {
    return tquat(a+q.a, b+q.b, c+q.c, d+q.d);
  }
  tquat& operator+=(const tquat &q) {
    a += q.a; b += q.b; c += q.c; d += q.d;
    return *this;
  }
  tquat operator*(const tquat &q) const {
    return tquat(
      a*q.a - b*q.b - c*q.c - d*q.d,
      a*q.b + b*q.a + c*q.d - d*q.c,
      a*q.c + c*q.a + d*q.b - b*q.d,
      a*q.d + d*q.a + b*q.c - c*q.b);
  }
  tquat& operator*=(const tquat &q) {
    return *this = *this * q;
  }
  tquat operator*(T s) const {
    return tquat(a*s, b*s, c*s, d*s);
  }
  tquat& operator*=(T s) {
    a*=s; b*=s; c*=s; d*=s;
    return *this;
  }
  tquat operator/(T s) const {
    return tquat(a/s, b/s, c/s, d/s);
  }

  void NormalizeMe() {
    T z = std::sqrt(a*a + b*b + c*c + d*d);
    a /= z;
    b /= z;
    c /= z;
    d /= z;
  }
  tquat Normalized() const {
    tquat res = *this;
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

  tquat Conjugate() const {
    return tquat(a, -b, -c, -d);
  }

  tquat Inverse() const {
    return Conjugate() / (a*a + b*b + c*c + d*d);
  }

  tvec3<T> Transform(tvec3<T> v) const {
    tquat q = *this * tquat(0, v.x, v.y, v.z) * this->Inverse();
    return tvec3<T>(q.b, q.c, q.d);
  }

  tvec3<T> Untransform(tvec3<T> v) const {
    tquat q = this->Inverse() * tquat(0, v.x, v.y, v.z) * *this;
    return tvec3<T>(q.b, q.c, q.d);
  }
};

template<typename T>
inline std::ostream& operator<<(std::ostream& o, const tquat<T>& q){
  return o<<'('<<q.a<<','<<q.b<<','<<q.c<<','<<q.d<<')';
}

using fquat = tquat<float>;
using dquat = tquat<double>;
