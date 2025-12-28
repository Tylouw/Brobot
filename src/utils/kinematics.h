#pragma once

#include "utils/types.h"

/*
  Kinematics API (project-wide)
  -----------------------------
  Units:
    - Joint angles: radians
    - Pose position: meters
    - Rotation: Mat3

  Conventions:
    - Pose is base->TCP
    - Jacobian is geometric Jacobian mapping qdot -> [v; w]
*/

#ifdef __cplusplus
extern "C" {
#endif

// Forward kinematics returning TCP pose and also intermediate frames
// o[i] = origin of frame i in base coords (meters)
// z[i] = z-axis of frame i in base coords (unit vector)
void fk_with_frames(const float q_rad[6], Pose& tcp, Vec3 o[6], Vec3 z[6]);

// Geometric Jacobian from frame origins/axes
// Input: o,z from fk_with_frames and tcp position p_e
// Output: J 6x6, rows: [vx vy vz wx wy wz]
void jacobian_from_frames(const Vec3 o[6], const Vec3 z[6], const Vec3& p_e, float J[6][6]);

// Analytical IK: Pose -> JointVector (radians)
// Returns one solution (your branch choice rules apply).
JointVector ik6(const Pose& target);

#ifdef __cplusplus
}
#endif
