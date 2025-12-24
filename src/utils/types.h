struct CartesianVelocity {
    Vec3 v;   // linear velocity [m/s]
    Vec3 w;   // angular velocity [rad/s]
};

struct Vec3 {
    float x, y, z;
};

struct Mat3 {
    float m[3][3];
};

struct Pose {
    Vec3 p;
    Mat3 R;
};

struct CartesianWaypoint {
    Pose pose;
    float blend_radius;  // meters
};
