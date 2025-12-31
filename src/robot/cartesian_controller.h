#pragma once

#include "utils/types.h"

class MotionController;

/*
  CartesianController
  -------------------
  Internal units only:
    - Pose position: meters
    - Rotations: Mat3
    - Joints: radians
    - Velocities: m/s, rad/s
*/
class CartesianController {
public:
    explicit CartesianController(MotionController* motion);

    void update();

    // Pose IK move (analytical IK once)
    bool moveToPose(const Pose& target_m,
                    float maxVelRad,
                    float accelRad);

    // Cartesian jogging (continuous)
    void setCartesianJog(const CartesianVelocity& cmd_mps_radps,
                         float jointAccelRad);

    void stopJog();

    const RobotState& state() const { return state_; }

    // Optional tuning
    void setDampingLambda(float lambda) { lambda_ = lambda; }

    // TCP config (flange->tcp)
    void setTCP(const Pose& T_flange_tcp);
    const Pose& getTCP() const;

    // TCP-aware state
    Pose tcpPose() const { return tcpPose_; } // base->tcp


private:
    MotionController* motion_;
    RobotState state_;

    bool jogActive_;
    CartesianVelocity jogCmd_;
    float jogJointAccelRad_;

    float lambda_; // DLS damping

    void refreshStateFromMotion();
    void updateFK();
    void computeJogJointVelTCP(const CartesianVelocity& cmd, float out_qdot[6]) const;

    static bool solve6x6(float A[6][6], const float b[6], float x[6]);

    Pose T_flange_tcp_;   // configured TCP
    Pose tcpPose_;        // base->tcp (derived each update)
};
