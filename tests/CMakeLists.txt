add_executable(test_smoke test_smoke.cpp)
target_link_libraries(test_smoke GTest::gtest nlohmann_json::nlohmann_json)

# Note: test_smoke deliberately does NOT link eCAL/protobuf/Qt — it only
# tests the pure logic in nodes/common/{transform_math,config}.hpp.
# eCAL-dependent behavior (actual pub/sub) needs a running eCAL instance
# and belongs in integration testing, not here.

include(GoogleTest)
gtest_discover_tests(test_smoke)
