cmake_minimum_required(VERSION 3.16)
project(camera_tracking CXX)
set(CMAKE_CXX_STANDARD 17)

# Dependencies come from vcpkg.json (manifest mode) — run with:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=<vcpkg>/scripts/buildsystems/vcpkg.cmake
# See README.md "Setup" section.

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Qt6 COMPONENTS Widgets QUIET)   # frontend only
find_package(GTest REQUIRED)

enable_testing()
add_subdirectory(tests)

# --- shared proto ---
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/camera_tracking.proto)
add_library(camera_tracking_proto ${PROTO_SRCS})
target_link_libraries(camera_tracking_proto protobuf::libprotobuf)
target_include_directories(camera_tracking_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# --- NatNet SDK (required by backend/optitrack) ---
# Proprietary, not on vcpkg -- download from OptiTrack's site (free account
# required) and point CMake at the install root (contains include/ and lib/).
set(NATNET_SDK_DIR "" CACHE PATH "Path to the NatNet SDK install (contains include/ and lib/)")
if (NOT NATNET_SDK_DIR)
    message(FATAL_ERROR "NATNET_SDK_DIR is not set -- backend needs the NatNet SDK "
                         "(optitrack is merged into it, see backend/src/main.cpp). "
                         "Pass -DNATNET_SDK_DIR=<path> to cmake.")
endif()

# --- backend: optitrack + fanuc[stub] + transform_sync merged into one process ---
add_executable(backend backend/src/main.cpp)
target_link_libraries(backend camera_tracking_proto eCAL::core nlohmann_json::nlohmann_json
                       "${NATNET_SDK_DIR}/lib/NatNetLib.lib")
target_include_directories(backend PRIVATE nodes/common "${NATNET_SDK_DIR}/include")

# --- mulib (UI library, brought in as a git submodule under external/) ---
# One-time setup:
#   git submodule add git@code.siemens-energy.com:psc-development/p2d2/software/mulib.git external/mulib
#   git submodule update --init --recursive
#
# Guarded with EXISTS so the rest of the build still works before the
# submodule has been added/initialized (e.g. fresh clone, forgot --recursive).
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/mulib/CMakeLists.txt")
    # mulib ships its own Catch2 test suite (RCU_BUILD_TESTS ON by default);
    # skip building it here so consuming this project doesn't also require
    # a catch2 vcpkg dependency just to build the frontend.
    set(RCU_BUILD_TESTS OFF CACHE BOOL "Disable mulib's own tests when built as a subproject" FORCE)
    add_subdirectory(external/mulib)
    set(MULIB_AVAILABLE TRUE)
else()
    message(WARNING "external/mulib not found — run "
                     "'git submodule update --init --recursive'. "
                     "frontend target will be skipped.")
    set(MULIB_AVAILABLE FALSE)
endif()

# --- frontend (Qt + mulib) ---
# mulib's CMakeLists.txt exports four separate targets (rcu_core,
# rcu_middleware, rcu_occt, rcu_qt_widgets) — there is no umbrella "mulib"
# target, so link all four directly.
if (Qt6_FOUND AND MULIB_AVAILABLE)
    add_executable(frontend frontend/src/main.cpp)
    target_compile_features(frontend PRIVATE cxx_std_20)  # mulib's public headers require C++20
    target_link_libraries(frontend camera_tracking_proto eCAL::core Qt6::Widgets nlohmann_json::nlohmann_json
                           rcu_core rcu_middleware rcu_occt rcu_qt_widgets)
    target_include_directories(frontend PRIVATE nodes/common)
endif()

# --- cameratracking (launcher): spawns backend.exe + frontend.exe +
# logger_node.py together so running one exe starts the whole system.
# Win32-only (uses CreateProcess/console-ctrl APIs directly).
if (WIN32 AND Qt6_FOUND AND MULIB_AVAILABLE)
    add_executable(cameratracking launcher/src/main.cpp)
    target_compile_definitions(cameratracking PRIVATE
        CAMERA_TRACKING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()
