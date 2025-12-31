#include <Arduino.h>
#include "robot.h"

#include "hardware/hardware_init.h"
#include "motion/motion_controller.h"
#include "robot/cartesian_controller.h"
#include "utils/robot_math.h"

HardwareInit hardware;
static MotionController* motion = nullptr;
static CartesianController* cart = nullptr;

void Robot::init() {
    Serial.begin(115200);
    Serial.setTimeout(5);

    hardware.initPins();
    motion = hardware.initMotors_getMotionController();

    cart = new CartesianController(motion);
}

static bool readLine(String& line) {
    if (Serial.available() <= 0) return false;
    line = Serial.readStringUntil('\n');
    line.trim();
    return line.length() > 0;
}

static int splitCsv(const String& s, String out[], int maxParts) {
    int count = 0;
    int start = 0;
    while (count < maxParts) {
        int end = s.indexOf(',', start);
        if (end < 0) {
            out[count++] = s.substring(start);
            break;
        }
        out[count++] = s.substring(start, end);
        start = end + 1;
    }
    for (int i = 0; i < count; ++i) out[i].trim();
    return count;
}

void Robot::update() {
    if (!motion || !cart) return;

    String line;
    if (readLine(line)) {
        char c0 = line.charAt(0);

        // Command mode if first token is letter
        if ((c0 >= 'A' && c0 <= 'Z') || (c0 >= 'a' && c0 <= 'z')) {
            c0 = (char)toupper(c0);

            if (c0 == 'P') { // pose move
                // P,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg,maxVelDeg,accelDeg
                // p,280,150,200,127.28,0,127.28,30,60
                String parts[12];
                int n = splitCsv(line.substring(1), parts, 12);

                if (n >= 9) {
                    const float x_mm = parts[1].toFloat();
                    const float y_mm = parts[2].toFloat();
                    const float z_mm = parts[3].toFloat();

                    const float rx_deg = parts[4].toFloat();
                    const float ry_deg = parts[5].toFloat();
                    const float rz_deg = parts[6].toFloat();

                    const float vmax_deg = parts[7].toFloat();
                    const float acc_deg  = parts[8].toFloat();

                    Pose target = pose_from_mm_rotvecDeg(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg);

                    Serial.print("Moving to pose mm: ");
                    Serial.print(target.p.x); Serial.print(",");
                    Serial.print(target.p.y); Serial.print(",");
                    Serial.print(target.p.z); Serial.println(" rotmat: ");
                    //print the pure matrix
                    Serial.print(target.R.m[0][0]); Serial.print(",");
                    Serial.print(target.R.m[0][1]); Serial.print(",");
                    Serial.print(target.R.m[0][2]); Serial.println();
                    Serial.print(target.R.m[1][0]); Serial.print(",");
                    Serial.print(target.R.m[1][1]); Serial.print(",");
                    Serial.print(target.R.m[1][2]); Serial.println();
                    Serial.print(target.R.m[2][0]); Serial.print(",");
                    Serial.print(target.R.m[2][1]); Serial.print(",");
                    Serial.print(target.R.m[2][2]); Serial.println();

                    cart->moveToPose(target, deg2rad(vmax_deg), deg2rad(acc_deg));
                } else {
                    Serial.println("ERR: P expects 9 CSV values after 'P,'");
                }
            }
            else if (c0 == 'V') { // velocity jogging
                // V,vx_mmps,vy_mmps,vz_mmps,wx_degps,wy_degps,wz_degps,accelDeg
                // v,30,0,0,0,0,0,90
                String parts[12];
                int n = splitCsv(line.substring(1), parts, 12);

                if (n >= 7) {
                    const float vx = parts[1].toFloat();
                    const float vy = parts[2].toFloat();
                    const float vz = parts[3].toFloat();

                    const float wx = parts[4].toFloat();
                    const float wy = parts[5].toFloat();
                    const float wz = parts[6].toFloat();

                    const float acc_deg = parts[7].toFloat();

                    CartesianVelocity cmd = cartvel_from_mmps_degps(vx, vy, vz, wx, wy, wz);
                    cart->setCartesianJog(cmd, deg2rad(acc_deg));
                } else {
                    Serial.println("ERR: V expects 7 CSV values after 'V,'");
                }
            }
            else if (c0 == 'S') {
                cart->stopJog();
            }
            else if (c0 == 'Q') {
                const RobotState& st = cart->state();

                Serial.print("q_deg: ");
                for (int i = 0; i < 6; ++i) {
                    Serial.print(rad2deg(st.q.q[i]));
                    Serial.print(i < 5 ? "," : "\n");
                }

                Serial.print("tcp_mm: ");
                Serial.print(st.tcp.p.x * 1000.0f); Serial.print(",");
                Serial.print(st.tcp.p.y * 1000.0f); Serial.print(",");
                Serial.print(st.tcp.p.z * 1000.0f); Serial.println();
                Serial.print("tcp_rotvec_deg: ");
                Vec3 rvec = mat3_to_rotvec(st.tcp.R);
                Serial.print(rad2deg(rvec.x)); Serial.print(",");
                Serial.print(rad2deg(rvec.y)); Serial.print(",");
                Serial.print(rad2deg(rvec.z)); Serial.println();

                Serial.print("im at pose: ");
                Serial.print(st.tcp.p.x); Serial.print(",");
                Serial.print(st.tcp.p.y); Serial.print(",");
                Serial.print(st.tcp.p.z); Serial.println(" rotmat: ");
                //print the pure matrix
                Serial.print(st.tcp.R.m[0][0]); Serial.print(",");
                Serial.print(st.tcp.R.m[0][1]); Serial.print(",");
                Serial.print(st.tcp.R.m[0][2]); Serial.println();
                Serial.print(st.tcp.R.m[1][0]); Serial.print(",");
                Serial.print(st.tcp.R.m[1][1]); Serial.print(",");
                Serial.print(st.tcp.R.m[1][2]); Serial.println();
                Serial.print(st.tcp.R.m[2][0]); Serial.print(",");
                Serial.print(st.tcp.R.m[2][1]); Serial.print(",");
                Serial.print(st.tcp.R.m[2][2]); Serial.println();

                motion->printMotorPositions();
                motion->printMotorSteps();
            }
            else {
                Serial.print("got command: ");
                Serial.print(c0);
                Serial.println(" ERR: Unknown command. Use P,V,S,Q or 6 joint values.");
                float joint_vals_test[6] = {10,-30,60,34.7,4,-80};
                float motors_intermediate[6];
                float joints_after_conversion[6];
                motion->jointsToMotorsRad(joint_vals_test, motors_intermediate);
                motion->motorsToJointsRad(motors_intermediate, joints_after_conversion);
                Serial.print("Test conversion check:\nInput joints (deg): ");
                for (int i = 0; i < 6; ++i) {
                    Serial.print(joint_vals_test[i]);
                    Serial.print(i < 5 ? "," : "\n");
                }
            }
        } else {
            // Legacy: 6 comma-separated joint angles in degrees
            String parts[6];
            int n = splitCsv(line, parts, 6);
            if (n == 6) {
                float q_deg[6];
                for (int i = 0; i < 6; ++i){
                    Serial.print("Joint ");
                    Serial.print(i+1);
                    Serial.print(": ");
                    Serial.println(parts[i]);
                    q_deg[i] = parts[i].toFloat();
                }
                // Your existing motion API likely has this already:
                motion->moveJointsDeg(q_deg, 60.0f, 180.0f);
            } else {
                Serial.println("ERR: Expected 6 joint angles or a command.");
            }
        }
    }

    cart->update();
}
