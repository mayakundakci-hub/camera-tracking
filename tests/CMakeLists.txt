add_executable(test_smoke test_smoke.cpp)
target_link_libraries(test_smoke GTest::gtest nlohmann_json::nlohmann_json)
include(GoogleTest)
gtest_discover_tests(test_smoke)
