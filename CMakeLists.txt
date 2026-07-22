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

# --- shared middleware (eCAL pub/sub wrappers, see middleware/CMakeLists.txt) ---
add_subdirectory(middleware)

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
add_executable(backend
    backend/src/main.cpp
    backend/src/optitrack.cpp
    backend/src/fanuc.cpp
    backend/src/transform_sync.cpp
)
target_link_libraries(backend camera_tracking_proto eCAL::core nlohmann_json::nlohmann_json
                       camera_tracking_middleware
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
#
# --- Munera SDK (mulib + ludus) ---
# Layering: arena (data/transport) -> ludus (headless domain services) ->
# mulib (Qt/QML presentation). This app uses the top two:
#   mulib  -- Mu.Material/Mu.Cad QML + mu::core/mu::robot (viewport UI)
#   ludus  -- URDF import + forward kinematics (Ludus::Robot)
#
# Consumed as local source subprojects rather than as ports from the private
# Munera vcpkg registry. Both are under active development in the sibling
# checkout, and ports would force a commit+push+SHA-bump cycle per change; the
# registry's ludus ref also still predates the URDF importer this app needs.
# Switch to find_package(MuLib/Ludus CONFIG) once those ports are current.
if (Qt6_FOUND)
    set(MUNERA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Munera" CACHE PATH
        "Path to the Munera SDK checkout (contains mulib/ and ludus/)")
    foreach(_munera_repo mulib ludus)
        if (NOT EXISTS "${MUNERA_DIR}/${_munera_repo}/CMakeLists.txt")
            message(FATAL_ERROR
                "Munera '${_munera_repo}' not found at ${MUNERA_DIR}/${_munera_repo} -- "
                "clone the Munera SDK next to this repo, or pass -DMUNERA_DIR=<path>.")
        endif()
    endforeach()

    # mulib's defaults already cover core/geometry/model/cad/qml/robot plus the
    # QML plugins; only its Catch2 test suite needs forcing off so consuming it
    # doesn't drag in a catch2 dependency just to build the frontend.
    set(MU_BUILD_TESTS OFF CACHE BOOL "Disable mulib's own tests when built as a subproject" FORCE)
    add_subdirectory("${MUNERA_DIR}/mulib" "${CMAKE_BINARY_DIR}/mulib")

    # ludus: only the robot library. Core requires Arena and Occt requires
    # OpenCASCADE -- neither is used here, so keeping them off holds the extra
    # vcpkg surface to just urdfdom (see vcpkg.json).
    set(LUDUS_BUILD_CORE OFF CACHE BOOL "" FORCE)
    set(LUDUS_BUILD_OCCT OFF CACHE BOOL "" FORCE)
    set(LUDUS_BUILD_ROBOT ON CACHE BOOL "" FORCE)
    set(LUDUS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    add_subdirectory("${MUNERA_DIR}/ludus" "${CMAKE_BINARY_DIR}/ludus")

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
