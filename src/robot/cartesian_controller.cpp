#include "robot/cartesian_controller.h"

#include <math.h>
#include "motion/motion_controller.h"
#include "utils/kinematics.h"
#include "utils/robot_math.h"

CartesianController::CartesianController(MotionController* motion)
: motion_(motion),
  state_{},
  jogActive_(false),
  jogCmd_{},
  jogJointAccelRad_(0.0f),
  lambda_(0.02f)
{
    T_flange_tcp_.R = mat3_identity();
    T_flange_tcp_.p = {0.0f, 0.0f, 0.0f};
    tcpPose_ = {};

    if (motion_) {
        motion_->getJointAnglesRad(state_.q.q);
        for (int i = 0; i < 6; ++i) state_.q_dot.q[i] = 0.0f;
        updateFK();
    }
}

void CartesianController::update() {
    if (!motion_) return;

    // Always advance motor timing and hardware updates
    motion_->update();

    // Refresh q (Stage 2 will ensure this reflects reality even while moving/jogging)
    refreshStateFromMotion();

    // FK for current TCP
    updateFK();

    // If jogging active, compute qdot and send it every cycle
    if (jogActive_) {
        float qdot[6] = {0};
        computeJogJointVel(jogCmd_, qdot);

        for (int i = 0; i < 6; ++i) state_.q_dot.q[i] = qdot[i];

        motion_->setJogJointVelRad(qdot, jogJointAccelRad_);
    }
}

bool CartesianController::moveToPose(const Pose& target_m,
                                    float maxVelRad,
                                    float accelRad)
{
    if (!motion_) return false;

    stopJog();
    refreshStateFromMotion();

    // Analytical IK
    Pose T_base_flange_target = pose_mul(target_m, pose_inv(T_flange_tcp_)); //adjust for tcp
    JointVector sol = ik6(T_base_flange_target);

    motion_->moveJointsRad(sol.q, maxVelRad, accelRad);
    return true;
}

void CartesianController::setCartesianJog(const CartesianVelocity& cmd_mps_radps,
                                         float jointAccelRad)
{
    jogCmd_ = cmd_mps_radps;
    jogJointAccelRad_ = jointAccelRad;
    jogActive_ = true;
}

void CartesianController::stopJog() {
    jogActive_ = false;
    jogCmd_ = {};
    for (int i = 0; i < 6; ++i) state_.q_dot.q[i] = 0.0f;

    if (motion_) motion_->stopJog();
}

void CartesianController::refreshStateFromMotion() {
    if (!motion_) return;
    motion_->getJointAnglesRad(state_.q.q);
}

void CartesianController::updateFK() {
    Pose ee{};
    Vec3 o[6];
    Vec3 z[6];
    fk_with_frames(state_.q.q, ee, o, z);
    tcpPose_ = pose_mul(ee, T_flange_tcp_);
    state_.tcp = tcpPose_;

}

void CartesianController::computeJogJointVel(const CartesianVelocity& cmd_tcp,
                                             float out_qdot[6]) const
{
    Pose ee{};
    Vec3 o[6];
    Vec3 z[6];
    fk_with_frames(state_.q.q, ee, o, z);

    float J[6][6];
    jacobian_from_frames(o, z, ee.p, J);

    const Vec3 r_base = mat3_mul_vec(ee.R, T_flange_tcp_.p);

    // Convert commanded TCP twist to flange twist:
    // w_F = w_T
    // v_F = v_T - w x r
    const Vec3 w = cmd_tcp.w;
    const Vec3 wxr = vec3_cross(w, r_base);
    const Vec3 vF = vec3_sub(cmd_tcp.v, wxr);

    float xdotF[6] = { vF.x, vF.y, vF.z, w.x, w.y, w.z };

    // A = J J^T + λ^2 I
    float A[6][6] = {0};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 6; ++k) s += J[i][k] * J[j][k];
            A[i][j] = s;
        }
    }
    const float l2 = lambda_ * lambda_;
    for (int i = 0; i < 6; ++i) A[i][i] += l2;

    float y[6] = {0};
    if (!solve6x6(A, xdotF, y)) {
        for (int i = 0; i < 6; ++i) out_qdot[i] = 0.0f;
        return;
    }

    // qdot = J^T y
    for (int j = 0; j < 6; ++j) {
        float s = 0.0f;
        for (int i = 0; i < 6; ++i) s += J[i][j] * y[i];
        out_qdot[j] = s;
    }
}

bool CartesianController::solve6x6(float A[6][6], const float b[6], float x[6]) {
    float M[6][7];
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) M[i][j] = A[i][j];
        M[i][6] = b[i];
    }

    for (int col = 0; col < 6; ++col) {
        int pivot = col;
        float maxAbs = fabsf(M[col][col]);
        for (int r = col + 1; r < 6; ++r) {
            float v = fabsf(M[r][col]);
            if (v > maxAbs) { maxAbs = v; pivot = r; }
        }
        if (maxAbs < 1e-9f) return false;

        if (pivot != col) {
            for (int c = col; c < 7; ++c) {
                float tmp = M[col][c];
                M[col][c] = M[pivot][c];
                M[pivot][c] = tmp;
            }
        }

        const float inv = 1.0f / M[col][col];
        for (int c = col; c < 7; ++c) M[col][c] *= inv;

        for (int r = 0; r < 6; ++r) {
            if (r == col) continue;
            const float f = M[r][col];
            for (int c = col; c < 7; ++c) M[r][c] -= f * M[col][c];
        }
    }

    for (int i = 0; i < 6; ++i) x[i] = M[i][6];
    return true;
}


void CartesianController::setTCP(const Pose& T_flange_tcp) {
    T_flange_tcp_ = T_flange_tcp;
}
const Pose& CartesianController::getTCP() const {
    return T_flange_tcp_;
}
