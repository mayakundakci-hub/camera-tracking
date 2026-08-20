include(GoogleTest)

add_executable(test_smoke test_smoke.cpp)
target_link_libraries(
    test_smoke
    PRIVATE
        GTest::gtest
        GTest::gtest_main
        nlohmann_json::nlohmann_json
)
gtest_discover_tests(test_smoke)

# Frame math. Nothing downstream is trustworthy if these fail -- a reversed
# transform renders a plausible scene and silently biases every error number.
add_executable(test_frames test_frames.cpp)
target_link_libraries(test_frames GTest::gtest GTest::gtest_main Eigen3::Eigen)
target_include_directories(test_frames PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
gtest_discover_tests(test_frames)

# Projecting a measured pose onto a revolute joint. The hand's J8/J9/J10 feature
# is essentially one function, so this is that feature's correctness argument --
# including a brute-force scan, which is the only test here that distinguishes
# the closed form from a plausible-but-suboptimal formula.
add_executable(test_joint_projection test_joint_projection.cpp)
target_link_libraries(test_joint_projection GTest::gtest GTest::gtest_main Eigen3::Eigen)
target_include_directories(test_joint_projection PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
gtest_discover_tests(test_joint_projection)

# Building one part's pose from two rigid bodies on it -- the rail and the rotor,
# same operation. Their origins are metres from either body, so this is where a
# tenth of a degree turns into millimetres, and the tests pin exactly which part
# of that error the chord between the bodies removes and which part survives.
add_executable(test_two_mount_pose test_two_mount_pose.cpp)
target_link_libraries(test_two_mount_pose GTest::gtest GTest::gtest_main Eigen3::Eigen)
target_include_directories(test_two_mount_pose PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
gtest_discover_tests(test_two_mount_pose)

# Scene manifest loading + validation. Most of these assert that a bad config
# is REFUSED -- a silently-defaulted scene renders fine and validates nothing.
add_executable(test_scene_config test_scene_config.cpp)
target_link_libraries(test_scene_config
    GTest::gtest GTest::gtest_main Eigen3::Eigen nlohmann_json::nlohmann_json)
target_include_directories(test_scene_config PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
gtest_discover_tests(test_scene_config)

# Measurements -> frames. Includes the test for the claim the whole anchoring
# scheme rests on: that Motive's world origin cancels out.
add_executable(test_object_placement test_object_placement.cpp)
target_link_libraries(test_object_placement
    GTest::gtest GTest::gtest_main Eigen3::Eigen nlohmann_json::nlohmann_json)
target_include_directories(test_object_placement PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
gtest_discover_tests(test_object_placement)

# The project's headline numbers. Includes the timing case: two continuously
# tracked placements sampled 20ms apart, with the hand moving, produce 10mm of
# pure timing error that looks exactly like a tracking error.
add_executable(test_comparison test_comparison.cpp)
target_link_libraries(test_comparison
    GTest::gtest GTest::gtest_main Eigen3::Eigen nlohmann_json::nlohmann_json)
target_include_directories(test_comparison PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
gtest_discover_tests(test_comparison)
