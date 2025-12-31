#include "utils/robot_math.h"

// ------- Mat3 basics -------
Mat3 mat3_identity() {
    Mat3 R{};
    R.m[0][0] = 1.0f; R.m[0][1] = 0.0f; R.m[0][2] = 0.0f;
    R.m[1][0] = 0.0f; R.m[1][1] = 1.0f; R.m[1][2] = 0.0f;
    R.m[2][0] = 0.0f; R.m[2][1] = 0.0f; R.m[2][2] = 1.0f;
    return R;
}

Mat3 mat3_mul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 3; ++k) s += A.m[i][k] * B.m[k][j];
            C.m[i][j] = s;
        }
    }
    return C;
}

Vec3 mat3_mul_vec(const Mat3& R, const Vec3& v) {
    return {
        R.m[0][0]*v.x + R.m[0][1]*v.y + R.m[0][2]*v.z,
        R.m[1][0]*v.x + R.m[1][1]*v.y + R.m[1][2]*v.z,
        R.m[2][0]*v.x + R.m[2][1]*v.y + R.m[2][2]*v.z
    };
}

Mat3 mat3_transpose(const Mat3& R) {
    Mat3 Rt{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Rt.m[i][j] = R.m[j][i];
    return Rt;
}

// ------- SO(3): exp/log -------
Mat3 so3_exp(const Vec3& r) {
    const float theta = vec3_norm(r);
    Mat3 R = mat3_identity();

    if (theta < 1e-9f) {
        // R ≈ I + [r]x
        const float rx = r.x, ry = r.y, rz = r.z;
        R.m[0][1] = -rz; R.m[0][2] =  ry;
        R.m[1][0] =  rz; R.m[1][2] = -rx;
        R.m[2][0] = -ry; R.m[2][1] =  rx;
        return R;
    }

    const float ux = r.x / theta;
    const float uy = r.y / theta;
    const float uz = r.z / theta;

    const float c = cosf(theta);
    const float s = sinf(theta);
    const float v = 1.0f - c;

    R.m[0][0] = c + ux*ux*v;
    R.m[0][1] = ux*uy*v - uz*s;
    R.m[0][2] = ux*uz*v + uy*s;

    R.m[1][0] = uy*ux*v + uz*s;
    R.m[1][1] = c + uy*uy*v;
    R.m[1][2] = uy*uz*v - ux*s;

    R.m[2][0] = uz*ux*v - uy*s;
    R.m[2][1] = uz*uy*v + ux*s;
    R.m[2][2] = c + uz*uz*v;

    return R;
}

Vec3 so3_log(const Mat3& R) {
    const float tr = R.m[0][0] + R.m[1][1] + R.m[2][2];
    float cos_theta = 0.5f * (tr - 1.0f);
    cos_theta = fmaxf(-1.0f, fminf(1.0f, cos_theta));

    const float theta = acosf(cos_theta);

    // Small angle: use first-order
    if (theta < 1e-6f) {
        return {
            0.5f * (R.m[2][1] - R.m[1][2]),
            0.5f * (R.m[0][2] - R.m[2][0]),
            0.5f * (R.m[1][0] - R.m[0][1])
        };
    }

    // Near 180°: use diagonal-based axis extraction
    if (M_PI - theta < 1e-3f) {
        float x2 = fmaxf(0.0f, (R.m[0][0] + 1.0f) * 0.5f);
        float y2 = fmaxf(0.0f, (R.m[1][1] + 1.0f) * 0.5f);
        float z2 = fmaxf(0.0f, (R.m[2][2] + 1.0f) * 0.5f);

        Vec3 axis{};
        if (x2 >= y2 && x2 >= z2) {
            axis.x = sqrtf(x2);
            axis.y = (R.m[0][1] + R.m[1][0]) / (4.0f * axis.x + 1e-9f);
            axis.z = (R.m[0][2] + R.m[2][0]) / (4.0f * axis.x + 1e-9f);
        } else if (y2 >= z2) {
            axis.y = sqrtf(y2);
            axis.x = (R.m[0][1] + R.m[1][0]) / (4.0f * axis.y + 1e-9f);
            axis.z = (R.m[1][2] + R.m[2][1]) / (4.0f * axis.y + 1e-9f);
        } else {
            axis.z = sqrtf(z2);
            axis.x = (R.m[0][2] + R.m[2][0]) / (4.0f * axis.z + 1e-9f);
            axis.y = (R.m[1][2] + R.m[2][1]) / (4.0f * axis.z + 1e-9f);
        }
        // Normalize in case of numeric noise
        const float n = vec3_norm(axis);
        if (n > 1e-6f) {
            axis = vec3_scale(axis, 1.0f / n);
        }
        return vec3_scale(axis, theta);
    }

    // General case
    const float denom = 2.0f * sinf(theta);
    const float k = theta / denom;
    return {
        k * (R.m[2][1] - R.m[1][2]),
        k * (R.m[0][2] - R.m[2][0]),
        k * (R.m[1][0] - R.m[0][1])
    };
}

// ------------------ USER EDGE conversions ------------------
Pose pose_from_mm_rotvecDeg(float x_mm, float y_mm, float z_mm,
                            float rx_deg, float ry_deg, float rz_deg)
{
    Pose T{};
    T.p = { x_mm / 1000.0f, y_mm / 1000.0f, z_mm / 1000.0f };

    // Rotation vector in radians:
    Vec3 rvec_rad{ deg2rad(rx_deg), deg2rad(ry_deg), deg2rad(rz_deg) };
    T.R = rotvec_to_mat3(rvec_rad);

    return T;
}

CartesianVelocity cartvel_from_mmps_degps(float vx_mmps, float vy_mmps, float vz_mmps,
                                         float wx_degps, float wy_degps, float wz_degps)
{
    CartesianVelocity cmd{};
    cmd.v = { vx_mmps / 1000.0f, vy_mmps / 1000.0f, vz_mmps / 1000.0f };
    cmd.w = { deg2rad(wx_degps), deg2rad(wy_degps), deg2rad(wz_degps) };
    return cmd;
}

Pose pose_mul(const Pose& A, const Pose& B) {
    Pose C{};
    C.R = mat3_mul(A.R, B.R);
    // p = A.p + A.R * B.p
    C.p = vec3_add(A.p, mat3_mul_vec(A.R, B.p));
    return C;
}

Pose pose_inv(const Pose& T) {
    Pose Ti{};
    Ti.R = mat3_transpose(T.R);
    // p_inv = -R^T * p
    Vec3 rp = mat3_mul_vec(Ti.R, T.p);
    Ti.p = vec3_scale(rp, -1.0f);
    return Ti;
}
