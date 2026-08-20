if(DEFINED ENV{ARENA_LOCAL_SOURCE_PATH})
  set(SOURCE_PATH "$ENV{ARENA_LOCAL_SOURCE_PATH}")
else()
  vcpkg_from_git(
    OUT_SOURCE_PATH
    SOURCE_PATH
    URL
    "git@code.siemens-energy.com:psc-development/p2d2/software/munera-sdk/arena.git"
    REF
    "2d5dad901a223ce59690cdabb7ead4b4c70f1051")
endif()

vcpkg_check_features(
  OUT_FEATURE_OPTIONS
  FEATURE_OPTIONS
  FEATURES
  model
  ARENA_BUILD_MODEL
  ecal
  ARENA_BUILD_ECAL)

vcpkg_cmake_configure(SOURCE_PATH "${SOURCE_PATH}" OPTIONS ${FEATURE_OPTIONS})
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME Arena CONFIG_PATH lib/cmake/Arena)
vcpkg_copy_pdbs()

# Ship the canonical language SDK sources with the native vcpkg package.
foreach(_language IN ITEMS dotnet python rust)
  if(EXISTS "${SOURCE_PATH}/sdk/${_language}")
    file(INSTALL "${SOURCE_PATH}/sdk/${_language}"
         DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}/sdk")
  endif()
endforeach()

file(
  WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/sdk-layout.json"
  "{\n"
  "  \"schema\": 1,\n"
  "  \"root\": \"share/${PORT}/sdk\",\n"
  "  \"dotnet\": \"share/${PORT}/sdk/dotnet\",\n"
  "  \"python\": \"share/${PORT}/sdk/python\",\n"
  "  \"rust\": \"share/${PORT}/sdk/rust\"\n"
  "}\n")

# vcpkg recognizes authored usage text only when it is installed here.
# Guarded: the registry copy of this port ships no `usage` file, so the
# unguarded file(INSTALL) upstream is a hard build failure.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/usage")
  file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage"
       DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include"
     "${CURRENT_PACKAGES_DIR}/debug/share")

# Install source licensing when available and retain an explicit package
# copyright artifact for internal revisions that predate a LICENSE file.
if(EXISTS "${SOURCE_PATH}/LICENSE")
  vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
elseif(EXISTS "${SOURCE_PATH}/LICENSE.txt")
  vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE.txt")
else()
  file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/copyright"
       "Arena source repository; see the repository licensing terms.\n")
endif()
