// =============================================================
// test_smoke.cpp
//
// Smoke tests for the two pieces of C++ logic worth catching bugs
// in before they reach a live camera/robot session:
//   - transform_math.hpp   (RigidTransform, loadCalibration, computeError)
//   - config.hpp            (Config::load / findConfigFile)
//
// Deliberately does NOT touch eCAL, NatNet, or Qt — those need real
// hardware/servers to test meaningfully and belong in integration
// testing, not here. This is what CAN be tested with zero hardware.
//
// Build & run (after vcpkg setup, see README):
//   cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
//   cmake --build build --target test_smoke
//   ./build/tests/test_smoke
// =============================================================

#include <gtest/gtest.h>
#include "../nodes/common/transform_math.hpp"
#include "../nodes/common/config.hpp"

#include <cstdio>
#include <fstream>

// ---------------- RigidTransform::apply ----------------

TEST(RigidTransform, IdentityTransformIsANoOp)
{
    RigidTransform t;  // default-constructed = identity, per transform_math.hpp
    double ox, oy, oz;
    t.apply(1.0, 2.0, 3.0, ox, oy, oz);
    EXPECT_DOUBLE_EQ(ox, 1.0);
    EXPECT_DOUBLE_EQ(oy, 2.0);
    EXPECT_DOUBLE_EQ(oz, 3.0);
}

TEST(RigidTransform, TranslationOnlyShiftsEveryAxis)
{
    RigidTransform t;
    t.T = {0.5, -1.0, 2.0};
    double ox, oy, oz;
    t.apply(0.0, 0.0, 0.0, ox, oy, oz);
    EXPECT_DOUBLE_EQ(ox, 0.5);
    EXPECT_DOUBLE_EQ(oy, -1.0);
    EXPECT_DOUBLE_EQ(oz, 2.0);
}

TEST(RigidTransform, NinetyDegreeRotationAboutZ)
{
    // Rotating (1,0,0) by +90deg about Z should land on (0,1,0)
    RigidTransform t;
    t.R = {{ {0, -1, 0}, {1, 0, 0}, {0, 0, 1} }};
    double ox, oy, oz;
    t.apply(1.0, 0.0, 0.0, ox, oy, oz);
    EXPECT_NEAR(ox, 0.0, 1e-9);
    EXPECT_NEAR(oy, 1.0, 1e-9);
    EXPECT_NEAR(oz, 0.0, 1e-9);
}

// ---------------- loadCalibration ----------------

TEST(LoadCalibration, ThrowsOnMissingFile)
{
    EXPECT_THROW(loadCalibration("/tmp/definitely_does_not_exist_p2d2.json"),
                 std::runtime_error);
}

TEST(LoadCalibration, ParsesRotationTranslationAndResidual)
{
    const char* path = "/tmp/p2d2_test_transform.json";
    {
        std::ofstream f(path);
        f << R"({
            "rotation": [[1,0,0],[0,1,0],[0,0,1]],
            "translation_m": [1.5, -2.5, 0.25],
            "registration_residual_mm": 0.83
        })";
    }

    RigidTransform t = loadCalibration(path);

    EXPECT_DOUBLE_EQ(t.R[0][0], 1.0);
    EXPECT_DOUBLE_EQ(t.T[0], 1.5);
    EXPECT_DOUBLE_EQ(t.T[1], -2.5);
    EXPECT_DOUBLE_EQ(t.T[2], 0.25);
    EXPECT_DOUBLE_EQ(t.residualMm, 0.83);

    std::remove(path);
}

TEST(LoadCalibration, MissingResidualDefaultsToNegativeOne)
{
    // registration_residual_mm omitted -> should default rather than throw,
    // since older/manual transform.json files may not have it yet
    const char* path = "/tmp/p2d2_test_transform_no_residual.json";
    {
        std::ofstream f(path);
        f << R"({
            "rotation": [[1,0,0],[0,1,0],[0,0,1]],
            "translation_m": [0,0,0]
        })";
    }

    RigidTransform t = loadCalibration(path);
    EXPECT_DOUBLE_EQ(t.residualMm, -1.0);

    std::remove(path);
}

// ---------------- computeError ----------------

TEST(ComputeError, ZeroWhenPointsMatchExactly)
{
    ErrorResult e = computeError(0.4, 0.1, 0.2,  0.4, 0.1, 0.2);
    EXPECT_DOUBLE_EQ(e.x_mm, 0.0);
    EXPECT_DOUBLE_EQ(e.y_mm, 0.0);
    EXPECT_DOUBLE_EQ(e.z_mm, 0.0);
    EXPECT_DOUBLE_EQ(e.total_mm, 0.0);
}

TEST(ComputeError, ConvertsMetersToMillimeters)
{
    // 1mm offset on X only (0.001 m)
    ErrorResult e = computeError(0.001, 0.0, 0.0,  0.0, 0.0, 0.0);
    EXPECT_NEAR(e.x_mm, 1.0, 1e-9);
    EXPECT_NEAR(e.total_mm, 1.0, 1e-9);
}

TEST(ComputeError, TotalIsEuclideanNormOfAxes)
{
    // 3-4-5 triangle scaled into mm-from-meters: 3mm, 4mm -> 5mm total
    ErrorResult e = computeError(0.003, 0.004, 0.0,  0.0, 0.0, 0.0);
    EXPECT_NEAR(e.total_mm, 5.0, 1e-9);
}

// ---------------- Config loader ----------------

TEST(Config, ThrowsClearlyWhenFileNotFound)
{
    EXPECT_THROW(Config::load("this_file_does_not_exist_anywhere.json"),
                 std::runtime_error);
}

// NOTE: Config::load() caches its result in a function-local `static`,
// so a "loads real config.json successfully" test would need to run in
// its own process (or Config would need a reset hook) to avoid bleeding
// state into other tests. Left as a TODO — worth adding once someone
// needs to test config-dependent node behavior specifically, e.g. via
// a small `--gtest_filter` in its own CI job rather than in this binary.

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
