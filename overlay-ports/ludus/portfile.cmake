if(DEFINED ENV{LUDUS_LOCAL_SOURCE_PATH})
  set(SOURCE_PATH "$ENV{LUDUS_LOCAL_SOURCE_PATH}")
else()
  vcpkg_from_git(
    OUT_SOURCE_PATH
    SOURCE_PATH
    URL
    "git@code.siemens-energy.com:psc-development/p2d2/software/munera-sdk/ludus.git"
    REF
    "59383946749912e2d1cbe7c117776164ad0f7d12")
endif()

vcpkg_check_features(
  OUT_FEATURE_OPTIONS
  FEATURE_OPTIONS
  FEATURES
  occt
  LUDUS_BUILD_OCCT
  robot
  LUDUS_BUILD_ROBOT)

vcpkg_cmake_configure(SOURCE_PATH "${SOURCE_PATH}" OPTIONS ${FEATURE_OPTIONS})

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME Ludus CONFIG_PATH lib/cmake/Ludus)

# Upstream LudusConfig.cmake.in declares only urdfdom for the robot feature,
# but Ludus::Robot's link interface also references sdformat15::sdformat15,
# Eigen3::Eigen, and orocos-kdl. Without these find_dependency calls every
# consumer fails at generate time with "target ... was not found".
if("robot" IN_LIST FEATURES)
  vcpkg_replace_string(
    "${CURRENT_PACKAGES_DIR}/share/Ludus/LudusConfig.cmake"
    "  find_dependency(urdfdom CONFIG REQUIRED)"
    "  find_dependency(urdfdom CONFIG REQUIRED)
  find_dependency(sdformat15 CONFIG REQUIRED)
  find_dependency(Eigen3 CONFIG REQUIRED)
  find_dependency(orocos_kdl CONFIG REQUIRED)")
endif()

# Ship the canonical language SDK sources with the native vcpkg package.  This
# lets non-C++ consumers use the same version selected by the vcpkg registry
# without requiring a separate NuGet, Python, or Cargo feed.
foreach(_language IN ITEMS dotnet python rust)
  if(EXISTS "${SOURCE_PATH}/sdk/${_language}")
    file(INSTALL "${SOURCE_PATH}/sdk/${_language}"
         DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}/sdk")
  endif()
endforeach()

# A stable, machine-readable package layout for consumer build scripts.
file(
  WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/sdk-layout.json"
  "{\n"
  "  \"schema\": 1,\n"
  "  \"root\": \"share/${PORT}/sdk\",\n"
  "  \"dotnet\": \"share/${PORT}/sdk/dotnet\",\n"
  "  \"python\": \"share/${PORT}/sdk/python\",\n"
  "  \"rust\": \"share/${PORT}/sdk/rust\"\n"
  "}\n")
vcpkg_copy_pdbs()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include"
     "${CURRENT_PACKAGES_DIR}/debug/share")

file(
  INSTALL "${SOURCE_PATH}/LICENSE"
  DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}"
  RENAME copyright OPTIONAL)
if(NOT EXISTS "${CURRENT_PACKAGES_DIR}/share/${PORT}/copyright")
  file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/copyright"
       "Ludus source repository; see repository licensing terms.\n")
endif()

# Guarded: the registry copy of this port ships no `usage` file, so the
# unguarded file(INSTALL) upstream is a hard build failure.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/usage")
  file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage"
       DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
endif()
