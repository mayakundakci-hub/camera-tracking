cmake_minimum_required(VERSION 3.16)
project(camera_tracking CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Qt6 6.8 COMPONENTS Widgets Quick QuickControls2 Qml QUIET)   # frontend only
if (Qt6_FOUND)
    qt_standard_project_setup(REQUIRES 6.8)
endif()
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

# --- mulib (UI library, brought in as a git submodule under external/) ---
# One-time setup:
#   git submodule add git@code.siemens-energy.com:psc-development/p2d2/software/mulib.git external/mulib
#   git submodule update --init --recursive
#
# backend links mu::middleware now too (EcalProtoPublisher/EcalProtoSubscriber,
# see backend/src/main.cpp), not just frontend, so mulib is no longer optional
# -- fail fast here instead of backend failing later with a confusing "cannot
# find mu/middleware/EcalProtoTopic.hpp" compile error.
if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/mulib/CMakeLists.txt")
    message(FATAL_ERROR "external/mulib not found -- run "
                         "'git submodule update --init --recursive'.")
endif()
# mulib ships its own Catch2 test suite (MU_BUILD_TESTS); force it off so
# consuming this project doesn't also require a catch2 vcpkg dependency
# just to build the frontend.
set(MU_BUILD_TESTS OFF CACHE BOOL "Disable mulib's own tests when built as a subproject" FORCE)
add_subdirectory(external/mulib)

# --- backend: optitrack + fanuc[stub] + transform_sync merged into one process ---
add_executable(backend
    backend/src/main.cpp
    backend/src/optitrack.cpp
    backend/src/fanuc.cpp
    backend/src/transform_sync.cpp
)
target_link_libraries(backend camera_tracking_proto eCAL::core nlohmann_json::nlohmann_json
                       mu::core mu::middleware
                       "${NATNET_SDK_DIR}/lib/x64/NatNetLib.lib")
target_include_directories(backend PRIVATE nodes/common "${NATNET_SDK_DIR}/include")

# NatNet SDK isn't managed by vcpkg, so there's no applocal-deploy step for
# it -- we only linked against the import lib (NatNetLib.lib); the runtime
# NatNetLib.dll has to be copied next to backend.exe by hand, same problem
# as the Qt platform plugin above but for a manually-vendored SDK instead.
if (WIN32)
    add_custom_command(TARGET backend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${NATNET_SDK_DIR}/lib/x64/NatNetLib.dll"
            "$<TARGET_FILE_DIR:backend>/NatNetLib.dll"
        COMMENT "Copying NatNetLib.dll next to backend.exe"
        VERBATIM
    )
endif()

# --- frontend (QtQuick/QML, Material style via mulib's Mu.Material) ---
# Defined in its own frontend/CMakeLists.txt -- see the comment there for why
# (qmlimportscanner's -rootPath == this target's SOURCE_DIR).
# mulib itself was already pulled in above (backend needs it too now).
if (Qt6_FOUND)
    add_subdirectory(frontend)
endif()

# --- cameratracking (launcher): spawns backend.exe + frontend.exe +
# logger_node.py together so running one exe starts the whole system.
# Win32-only (uses CreateProcess/console-ctrl APIs directly).
if (WIN32 AND Qt6_FOUND)
    add_executable(cameratracking launcher/src/main.cpp)
    target_compile_definitions(cameratracking PRIVATE
        CAMERA_TRACKING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()
