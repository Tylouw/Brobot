#ifndef TYPES_H
#define TYPES_H

struct Vec3 {
    float x, y, z;
};

struct Mat3 {
    float m[3][3];
};

struct JointVector {
    float q[6];
};

struct CartesianVelocity {
    Vec3 v;   // linear velocity [m/s]
    Vec3 w;   // angular velocity [rad/s]
};

struct Pose {
    Vec3 p;
    Mat3 R;
};

struct RobotState {
    JointVector q;          // current joint positions [rad]
    JointVector q_dot;      // current joint velocities [rad/s]
    Pose tcp;               // FK(q)
};

struct CartesianWaypoint {
    Pose pose;
    float blend_radius;  // meters
};

const float PI_HALF = 1.57079632679f;  // π/2
// const float DEG_TO_RAD = 0.01745329252f; // π/180
// const float RAD_TO_DEG = 57.2957795131f; // 180

static const float DH_A[6]     = {0.04f, 0.20f, 0.0f, 0.0f, 0.0f, 0.0f};
static const float DH_ALPHA[6] = {PI_HALF, 0.0f, PI_HALF, -PI_HALF, PI_HALF, 0.0f};
static const float DH_D[6]     = {0.14f, 0.0f, 0.0f, 0.20f, 0.0f, 0.034f};
static const float DH_THETA_OFFSET[6] = {0.0f, PI_HALF, 0.0f, 0.0f, 0.0f, 0.0f};

#endif