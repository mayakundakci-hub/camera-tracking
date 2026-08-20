// Smoke tests for the shared config loader.
//
// This file used to also cover RigidTransform / loadCalibration / computeError
// from nodes/common/transform_math.hpp. That header backed transform_sync's
// fanuc-vs-optitrack path, which the scene comparison replaced -- the frame
// math now lives in frames.hpp (test_frames.cpp) and the error computation in
// comparison.hpp (test_comparison.cpp), both with far better coverage.

#include <gtest/gtest.h>

#include "../nodes/common/config.hpp"

TEST(Config, ThrowsClearlyWhenFileNotFound)
{
    EXPECT_THROW(Config::load("this_file_does_not_exist_anywhere.json"),
                 std::runtime_error);
}

// NOTE: Config::load() caches its result in a function-local `static`, so a
// "loads real config.json successfully" test would need to run in its own
// process (or Config would need a reset hook) to avoid bleeding state into
// other tests. Left as a TODO -- worth adding once someone needs to test
// config-dependent node behavior specifically, e.g. via a small --gtest_filter
// in its own CI job rather than in this binary.
