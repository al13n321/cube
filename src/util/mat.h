#pragma once

#include "vec.h"

struct fmat3 {
  float m[3 * 3];

  fmat3() = default;
  fmat3(float m0, float m1, float m2,
        float m3, float m4, float m5,
        float m6, float m7, float m8)
    : m{m0,m1,m2,m3,m4,m5,m6,m7,m8} {}

  fmat3 operator*(const fmat3& b) const;
  fmat3 operator*(float b) const;
  fmat3& operator*=(float b);
  fmat3 Inverse() const;

  static fmat3 Zero();
  static fmat3 Identity();
  static fmat3 Diag(float m0, float m4, float m8);
};

fmat3 operator*(float b, const fmat3& m);
fvec3 operator*(const fmat3& m, const fvec3& v);

struct fmat4 {
  float m[4 * 4];

  fmat4() = default;
  fmat4(float m0 , float m1 , float m2 , float m3,
        float m4 , float m5 , float m6 , float m7,
        float m8 , float m9 , float m10, float m11,
        float m12, float m13, float m14, float m15) {
    m[0] = m0;
    m[1] = m1;
    m[2] = m2;
    m[3] = m3;
    m[4] = m4;
    m[5] = m5;
    m[6] = m6;
    m[7] = m7;
    m[8] = m8;
    m[9] = m9;
    m[10] = m10;
    m[11] = m11;
    m[12] = m12;
    m[13] = m13;
    m[14] = m14;
    m[15] = m15;
  }
	
  fmat4 operator*(const fmat4 &b) const;

  inline fvec3 Transform (const fvec3 &v) const {
    float d = v.x*m[12] + v.y*m[13] + v.z*m[14] + m[15];
    return fvec3((v.x*m[0] + v.y*m[1] + v.z*m[2]  + m[3] ) / d,
                 (v.x*m[4] + v.y*m[5] + v.z*m[6]  + m[7] ) / d,
                 (v.x*m[8] + v.y*m[9] + v.z*m[10] + m[11]) / d);
  }

  fmat4 Inverse();
  fmat4 Transposed();

  static fmat4 Zero();
  static fmat4 Identity();
  static fmat4 Translation(fvec3 delta);
  static fmat4 RotationX(float yaw);
  static fmat4 RotationY(float pitch);
  static fmat4 RotationZ(float roll);
  static fmat4 Rotation(float yaw, float pitch, float roll);
  // fov - horizontal field of view in degrees
  // near_plane must be positive
  // far_plane must be either zero or strictly greater than near_plane; zero means infinite projection matrix (no far clip plane)
  static fmat4 PerspectiveProjection(float fov, float aspect_ratio, float near_plane, float far_plane);
};
