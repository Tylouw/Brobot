#include "kinematics.h"
#include "robot_math.h"
#include <cmath>

void fk_with_frames(
    const float q[6],
    Pose& ee,
    Vec3 o[6],
    Vec3 z[6]
) {
    // Initialize base frame
    Mat3 R = {{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
    Vec3 p = {0, 0, 0};

    for (int i = 0; i < 6; ++i) {
        // Store joint frame BEFORE applying joint i transform
        o[i] = p;
        z[i] = {R.m[0][2], R.m[1][2], R.m[2][2]};

        float theta = q[i] + DH_THETA_OFFSET[i];

        float ct = cosf(theta);
        float st = sinf(theta);
        float ca = cosf(DH_ALPHA[i]);
        float sa = sinf(DH_ALPHA[i]);

        // Rotation from frame i to i+1 (standard DH)
        Mat3 R_i = {{
            { ct, -st*ca,  st*sa },
            { st,  ct*ca, -ct*sa },
            { 0,      sa,     ca }
        }};

        // Translation in frame i
        Vec3 p_i = {
            DH_A[i] * ct,
            DH_A[i] * st,
            DH_D[i]
        };

        // Update world pose
        p = vec3_add(p, mat3_mul_vec(R, p_i));
        R = mat3_mul(R, R_i);
    }

    ee.p = p;
    ee.R = R;
}


void jacobian_from_frames(
    const Vec3 o[6],
    const Vec3 z[6],
    const Vec3& p_ee,
    float J[6][6]
) {
    for (int i = 0; i < 6; ++i) {
        Vec3 dp = {
            p_ee.x - o[i].x,
            p_ee.y - o[i].y,
            p_ee.z - o[i].z
        };

        Vec3 Jv = vec3_cross(z[i], dp);

        // Linear part
        J[0][i] = Jv.x;
        J[1][i] = Jv.y;
        J[2][i] = Jv.z;

        // Angular part
        J[3][i] = z[i].x;
        J[4][i] = z[i].y;
        J[5][i] = z[i].z;
    }
}

JointVector ik6(const Pose& T) {
    JointVector result;

    // Extract rotation
    float r11 = T.R.m[0][0], r12 = T.R.m[0][1], r13 = T.R.m[0][2];
    float r21 = T.R.m[1][0], r22 = T.R.m[1][1], r23 = T.R.m[1][2];
    float r31 = T.R.m[2][0], r32 = T.R.m[2][1], r33 = T.R.m[2][2];

    // Position
    float px = T.p.x;
    float py = T.p.y;
    float pz = T.p.z;

    // Robot constants
    const float d1 = DH_D[0];  // in mm
    const float a1 = DH_A[0];
    const float a2 = DH_A[1];
    const float d4 = DH_D[3];
    const float d6 = DH_D[5];
    // Wrist center
    float xc = px - d6 * r13;
    float yc = py - d6 * r23;
    float zc = pz - d6 * r33;

    float theta1 = atan2f(yc, xc);

    float d = sqrtf(xc*xc + yc*yc) - a1;
    float e = zc - d1;

    float c_sq = d*d + e*e;
    float c = sqrtf(c_sq);

    float theta2 =
        acosf((a2*a2 + c_sq - d4*d4) / (2.0f * a2 * c))
        + atan2f(e, d)
        - PI_HALF;

    float theta3 =
        acosf((d4*d4 + a2*a2 - c_sq) / (2.0f * d4 * a2))
        - PI_HALF;
    float c1 = cosf(theta1);
    float s1 = sinf(theta1);
    float c23 = cosf(theta2 + theta3);
    float s23 = sinf(theta2 + theta3);

    float m13 = -r13*c1*s23 - r23*s1*s23 + r33*c23;
    float m23 =  r13*s1     - r23*c1;
    float m33 =  r13*c1*c23 + r23*s1*c23 + r33*s23;
    float m31 =  r11*c1*c23 + r21*s1*c23 + r31*s23;
    float m32 =  r12*c1*c23 + r22*s1*c23 + r32*s23;

    float theta4 = atan2f(m23, m13);
    float theta5 = atan2f(sqrtf(1.0f - m33*m33), m33);
    float theta6 = atan2f(m32, -m31);

    result.q[0] = theta1;
    result.q[1] = theta2;
    result.q[2] = theta3;
    result.q[3] = theta4;
    result.q[4] = theta5;
    result.q[5] = theta6;

    return result;
}
