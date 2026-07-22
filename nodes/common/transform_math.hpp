#pragma once

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>

// ---- calibration transform: fanuc_base -> optitrack_world ----
// apply() maps a point from the FANUC base frame into the OptiTrack world
// frame, so validation compares both systems in the camera's (OptiTrack) frame.
struct RigidTransform {
    std::array<std::array<double,3>,3> R { { {1,0,0}, {0,1,0}, {0,0,1} } };
    std::array<double,3>               T { 0.0, 0.0, 0.0 };
    double residualMm = -1.0;   // -1 = not loaded yet / unknown

    void apply(double x, double y, double z, double& ox, double& oy, double& oz) const {
        ox = R[0][0]*x + R[0][1]*y + R[0][2]*z + T[0];
        oy = R[1][0]*x + R[1][1]*y + R[1][2]*z + T[1];
        oz = R[2][0]*x + R[2][1]*y + R[2][2]*z + T[2];
    }
    // TODO: also rotate the quaternion if orientation error is ever needed
};

// Loads the transform.json produced by calibration/compute_transform.py.
// Throws if the file is missing/malformed — a bad or missing calibration
// should be a loud startup failure, not a silent identity transform,
// since identity would make every error reading wrong without warning.
inline RigidTransform loadCalibration(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("Calibration: could not open " + path +
                                  " — run calibration/compute_transform.py first");

    nlohmann::json j;
    f >> j;

    RigidTransform t;
    const auto& rot = j.at("rotation");
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            t.R[r][c] = rot.at(r).at(c).get<double>();

    const auto& trans = j.at("translation_m");
    for (int i = 0; i < 3; ++i)
        t.T[i] = trans.at(i).get<double>();

    t.residualMm = j.value("registration_residual_mm", -1.0);
    return t;
}

// Result of comparing a transformed OptiTrack point against a Fanuc point.
struct ErrorResult {
    double x_mm, y_mm, z_mm, total_mm;
};

// Pure error computation: given two points in the SAME frame (now the
// optitrack_world frame -- the raw OptiTrack camera point, and the FANUC point
// transformed into it), both in meters, compute the per-axis and total error in
// millimeters. Extracted as its own function specifically so the
// "meters -> mm, then euclidean norm" logic has one tested implementation
// instead of being inlined in a lambda.
inline ErrorResult computeError(double cam_x, double cam_y, double cam_z,
                                 double fanuc_x, double fanuc_y, double fanuc_z)
{
    ErrorResult r;
    r.x_mm = (cam_x - fanuc_x) * 1000.0;
    r.y_mm = (cam_y - fanuc_y) * 1000.0;
    r.z_mm = (cam_z - fanuc_z) * 1000.0;
    r.total_mm = std::sqrt(r.x_mm*r.x_mm + r.y_mm*r.y_mm + r.z_mm*r.z_mm);
    return r;
}
