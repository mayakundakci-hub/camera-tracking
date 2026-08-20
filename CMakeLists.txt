cmake_minimum_required(VERSION 3.16)
project(camera_tracking CXX)

# 20 rather than 17 because Arena's targets declare `target_compile_features(...
# PUBLIC cxx_std_20)`, and PUBLIC propagates: anything linking Arena::TransportEcal
# or Arena::Model must itself compile as C++20.
#
# The frontend was already effectively there -- it links Ludus::Robot, which pulls
# Arena::Model -- so CMake was already raising that target's standard past the 17
# below. This makes it explicit and applies it to backend, logger_node, middleware
# and the tests, which is what the move to Arena's eCAL transport needs.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC's default COFF object format caps a translation unit at 65,279 sections,
# and a test file that instantiates gtest's machinery over Eigen types passes
# that without being large in any way a reader would notice --
# test_object_placement.cpp hit it as `fatal error C1128`.
#
# /bigobj lifts the cap. It changes the object file format only: no runtime cost,
# no code generation difference, and the linker has understood it for twenty
# years. Applied project-wide rather than to the one file that failed, because
# the limit is reached by accumulation and the next file to cross it gives the
# same opaque error somewhere else.
if (MSVC)
    add_compile_options(/bigobj)
endif()

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Qt6 6.8 COMPONENTS Widgets Quick QuickControls2 Qml QUIET)   # frontend only
if (Qt6_FOUND)
    qt_standard_project_setup(REQUIRES 6.8)
endif()
find_package(GTest REQUIRED)
# Header-only. Backs nodes/common/frames.hpp -- all transform arithmetic in
# this project is Eigen's, so every target that touches a pose links it.
find_package(Eigen3 CONFIG REQUIRED)

enable_testing()
add_subdirectory(tests)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/camera_tracking.proto)
add_library(camera_tracking_proto ${PROTO_SRCS})
target_link_libraries(camera_tracking_proto protobuf::libprotobuf)
target_include_directories(camera_tracking_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# --- Munera ---
#
# How mulib, ludus and arena are consumed:
#   OFF (default) -- local ../Munera source via add_subdirectory. Use this while
#                    iterating; picks up uncommitted local changes.
#   ON            -- installed vcpkg-registry ports. Requires the manifest
#                    feature 'munera-registry' (VCPKG_MANIFEST_FEATURES) and a
#                    populated registry, which means every change to Munera has
#                    been committed, pushed, and had its port REF bumped. What
#                    the registry builds is what was PUSHED, so a local edit that
#                    is not pushed silently does not exist in this mode.
#
# Declared here rather than beside the frontend because ARENA is needed by the
# backend and the logger, which build with no Qt at all. Only mulib and ludus are
# frontend-only, and they stay behind the Qt guard further down.
option(MUNERA_FROM_REGISTRY
    "Consume Munera from the private vcpkg registry instead of local ../Munera source" OFF)
set(MUNERA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Munera" CACHE PATH
    "Path to the Munera SDK checkout (contains arena/, mulib/ and ludus/)")

# Arena supplies the eCAL transport this project used to carry itself -- see
# middleware/CMakeLists.txt.
#
# MODEL is on even though no camera-tracking source includes <arena/model/...>:
# mulib's mu_model, mu_cad and mu_robot all link Arena::Model PUBLIC, so mulib
# will not configure without it. It costs two static libraries over protobuf,
# which is already a dependency. ABI is genuinely unused and stays off.
if (MUNERA_FROM_REGISTRY)
    find_package(Arena CONFIG REQUIRED)
else()
    if (NOT EXISTS "${MUNERA_DIR}/arena/CMakeLists.txt")
        message(FATAL_ERROR
            "Munera 'arena' not found at ${MUNERA_DIR}/arena -- clone the Munera SDK "
            "next to this repo, or pass -DMUNERA_DIR=<path>.")
    endif()
    set(ARENA_BUILD_MODEL ON  CACHE BOOL "" FORCE)
    set(ARENA_BUILD_ECAL  ON  CACHE BOOL "" FORCE)
    set(ARENA_BUILD_ABI   OFF CACHE BOOL "" FORCE)
    add_subdirectory("${MUNERA_DIR}/arena" "${CMAKE_BINARY_DIR}/arena")

    # mulib's own root CMakeLists calls find_package(Arena CONFIG REQUIRED),
    # which has nothing to find in local-source mode: arena is a subdirectory of
    # THIS build, so it defines Arena::Model / Arena::TransportEcal directly and
    # never produces an installed ArenaConfig.cmake for find_package to locate.
    #
    # Point Arena_DIR at a generated config that only asserts the in-tree targets
    # are there. Same idea as CMAKE_FIND_PACKAGE_REDIRECTS_DIR (CMake 3.24+,
    # which FetchContent uses for exactly this situation), written by hand so it
    # depends on neither the CMake version nor FetchContent.
    set(_arena_shim_dir "${CMAKE_BINARY_DIR}/arena-find-shim")
    file(MAKE_DIRECTORY "${_arena_shim_dir}")
    file(WRITE "${_arena_shim_dir}/ArenaConfig.cmake" [==[
# Generated by camera-tracking's top-level CMakeLists. Arena is built in-tree via
# add_subdirectory; this exists only so a subproject's find_package(Arena)
# resolves to those already-defined targets instead of failing.
foreach(_t Arena::Protocol Arena::Model Arena::TransportEcal)
  if (NOT TARGET ${_t})
    message(FATAL_ERROR
      "Arena is expected in-tree but ${_t} is undefined -- check the "
      "ARENA_BUILD_* settings in camera-tracking/CMakeLists.txt.")
  endif()
endforeach()
set(Arena_FOUND TRUE)
]==])
    set(Arena_DIR "${_arena_shim_dir}" CACHE PATH
        "In-tree Arena shim -- see the Munera block in CMakeLists.txt" FORCE)
endif()

add_subdirectory(middleware)
set(NATNET_SDK_DIR "" CACHE PATH "Path to the NatNet SDK install (contains include/ and lib/)")
if (NOT NATNET_SDK_DIR)
    message(FATAL_ERROR "NATNET_SDK_DIR is not set -- backend needs the NatNet SDK "
                         "(optitrack is merged into it, see backend/src/main.cpp). "
                         "Pass -DNATNET_SDK_DIR=<path> to cmake.")
endif()

add_executable(backend
    backend/src/main.cpp
    backend/src/optitrack.cpp
    backend/src/fanuc.cpp
)
target_link_libraries(backend camera_tracking_proto eCAL::core nlohmann_json::nlohmann_json
                       camera_tracking_middleware Eigen3::Eigen
                       "${NATNET_SDK_DIR}/lib/x64/NatNetLib.lib")
target_include_directories(backend PRIVATE nodes/common "${NATNET_SDK_DIR}/include")

if (WIN32)
    add_custom_command(TARGET backend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${NATNET_SDK_DIR}/lib/x64/NatNetLib.dll"
            "$<TARGET_FILE_DIR:backend>/NatNetLib.dll"
        COMMENT "Copying NatNetLib.dll next to backend.exe"
        VERBATIM
    )
endif()

add_executable(logger_node nodes/logger_node/logger_node.cpp)
# camera_tracking_middleware brings eCAL + protobuf usage requirements (PUBLIC)
target_link_libraries(logger_node camera_tracking_proto nlohmann_json::nlohmann_json
                       camera_tracking_middleware Eigen3::Eigen)
target_include_directories(logger_node PRIVATE nodes/common)

if (Qt6_FOUND)
    # mulib and ludus stay behind this guard because only the frontend needs
    # them: the backend does no URDF reading -- a projected placement's joint
    # geometry is measured CAD written straight into config, not derived from a
    # robot description. Arena is the exception and is handled above, since the
    # backend and logger need its transport and build without Qt.
    #
    # MUNERA_FROM_REGISTRY and MUNERA_DIR are both declared above.
    if (MUNERA_FROM_REGISTRY)
        # Both projects set EXPORT_NAME, so the installed packages give MuLib::Core,
        # MuLib::Robot and Ludus::Robot -- the same spellings the in-tree ALIAS
        # targets use, which is what keeps frontend/CMakeLists.txt identical in
        # either mode. find_package also defines MuLib_QML_IMPORT_DIR_DEBUG /
        # _RELEASE for the QML import path.
        find_package(MuLib CONFIG REQUIRED)
        find_package(Ludus CONFIG REQUIRED)
    else()
        set(MUNERA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../Munera" CACHE PATH
            "Path to the Munera SDK checkout (contains mulib/ and ludus/)")
        foreach(_munera_repo mulib ludus)
            if (NOT EXISTS "${MUNERA_DIR}/${_munera_repo}/CMakeLists.txt")
                message(FATAL_ERROR
                    "Munera '${_munera_repo}' not found at ${MUNERA_DIR}/${_munera_repo} -- "
                    "clone the Munera SDK next to this repo, or pass -DMUNERA_DIR=<path>.")
            endif()
        endforeach()

        set(MU_BUILD_TESTS OFF CACHE BOOL "Disable mulib's own tests when built as a subproject" FORCE)
        add_subdirectory("${MUNERA_DIR}/mulib" "${CMAKE_BINARY_DIR}/mulib")
        set(LUDUS_BUILD_CORE OFF CACHE BOOL "" FORCE)
        set(LUDUS_BUILD_OCCT OFF CACHE BOOL "" FORCE)
        set(LUDUS_BUILD_ROBOT ON CACHE BOOL "" FORCE)
        set(LUDUS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        # ABI defaults ON and hard-errors without LUDUS_BUILD_CORE, which is off
        # above -- nothing here calls the C ABI.
        set(LUDUS_BUILD_ABI OFF CACHE BOOL "" FORCE)
        add_subdirectory("${MUNERA_DIR}/ludus" "${CMAKE_BINARY_DIR}/ludus")

        # Present the same target + variable names the installed package would, so
        # frontend/CMakeLists.txt is byte-for-byte identical in both modes. Both
        # are no-ops against a mulib that already declares them; they exist for
        # older checkouts that only define the bare mu_core / mu_robot targets.
        if (NOT TARGET MuLib::Core)
            add_library(MuLib::Core ALIAS mu_core)
        endif()
        if (NOT TARGET MuLib::Robot)
            add_library(MuLib::Robot ALIAS mu_robot)
        endif()
        set(MuLib_QML_IMPORT_DIR_DEBUG "${CMAKE_BINARY_DIR}/qml")
        set(MuLib_QML_IMPORT_DIR_RELEASE "${CMAKE_BINARY_DIR}/qml")
    endif()

    add_subdirectory(frontend)
endif()

if (WIN32 AND Qt6_FOUND)
    add_executable(cameratracking launcher/src/main.cpp)
    target_compile_definitions(cameratracking PRIVATE
        CAMERA_TRACKING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
endif()
