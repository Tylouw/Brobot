#pragma once
#include <Arduino.h>
#include <math.h>
#include "utils/types.h"

// ------------------ Basic helpers ------------------
inline float deg2rad(float d) { return d * (float)DEG_TO_RAD; }
inline float rad2deg(float r) { return r * (float)RAD_TO_DEG; }

// -------- Vec3 helpers --------
inline Vec3 vec3_add(const Vec3& a, const Vec3& b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
inline Vec3 vec3_sub(const Vec3& a, const Vec3& b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
inline Vec3 vec3_scale(const Vec3& v, float s)      { return {v.x*s, v.y*s, v.z*s}; }

inline float vec3_dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3  vec3_cross(const Vec3& a, const Vec3& b) {
    return { a.y*b.z - a.z*b.y,
             a.z*b.x - a.x*b.z,
             a.x*b.y - a.y*b.x };
}
inline float vec3_norm(const Vec3& v) { return sqrtf(vec3_dot(v,v)); }

// -------- Mat3 helpers --------
Mat3 mat3_identity();
Mat3 mat3_mul(const Mat3& A, const Mat3& B);
Vec3 mat3_mul_vec(const Mat3& R, const Vec3& v);
Mat3 mat3_transpose(const Mat3& R);

// -------- SO(3) helpers --------
// rotvec [rad] -> rotation matrix
Mat3 so3_exp(const Vec3& rotvec_rad);
// rotation matrix -> rotvec [rad]
Vec3 so3_log(const Mat3& R);

inline Mat3 rotvec_to_mat3(const Vec3& rotvec_rad) { return so3_exp(rotvec_rad); }
inline Vec3 mat3_to_rotvec(const Mat3& R)          { return so3_log(R); }

// ------------------ USER EDGE conversions ------------------
// These are intentionally explicit about units.
// Output types are ALWAYS in internal units: meters, radians.

Pose pose_from_mm_rotvecDeg(float x_mm, float y_mm, float z_mm,
                            float rx_deg, float ry_deg, float rz_deg);

CartesianVelocity cartvel_from_mmps_degps(float vx_mmps, float vy_mmps, float vz_mmps,
                                         float wx_degps, float wy_degps, float wz_degps);

Pose pose_mul(const Pose& A, const Pose& B);
Pose pose_inv(const Pose& T);
