if(DEFINED ENV{MULIB_LOCAL_SOURCE_PATH})
  set(SOURCE_PATH "$ENV{MULIB_LOCAL_SOURCE_PATH}")
else()
  vcpkg_from_git(
    OUT_SOURCE_PATH
    SOURCE_PATH
    URL
    "git@code.siemens-energy.com:psc-development/p2d2/software/munera-sdk/mulib.git"
    REF
    "71a3bfdd2251510efe050f3e6936129a5e66c14f")
endif()

vcpkg_check_features(
  OUT_FEATURE_OPTIONS
  FEATURE_OPTIONS
  FEATURES
  qml
  MU_BUILD_QML
  qml
  MU_BUILD_QML_PLUGINS
  qml
  MU_INSTALL_QML_IMPORTS
  cad
  MU_BUILD_CAD
  robot
  MU_BUILD_ROBOT)

set(_build_geometry OFF)
set(_build_model OFF)
if("qml" IN_LIST FEATURES)
  set(_build_geometry ON)
endif()
if("qml" IN_LIST FEATURES
   OR "cad" IN_LIST FEATURES
   OR "robot" IN_LIST FEATURES)
  set(_build_model ON)
endif()
if("robot" IN_LIST FEATURES AND NOT "qml" IN_LIST FEATURES)
  message(
    FATAL_ERROR
      "The mulib[robot] feature requires mulib[qml]. Request both features.")
endif()

vcpkg_cmake_configure(
  SOURCE_PATH
  "${SOURCE_PATH}"
  OPTIONS
  ${FEATURE_OPTIONS}
  -DMU_BUILD_CORE=ON
  -DMU_BUILD_GEOMETRY=${_build_geometry}
  -DMU_BUILD_MODEL=${_build_model}
  -DMU_BUILD_QT_WIDGETS=OFF)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(PACKAGE_NAME MuLib CONFIG_PATH lib/cmake/MuLib)
vcpkg_copy_pdbs()

# Upstream MuLibConfig.cmake computes a single, config-invariant
# MuLib_QML_IMPORT_DIR by walking up three directory levels from share/mulib/
# (-> share -> <triplet> -> vcpkg root), one hop too many: it lands on the
# vcpkg install root instead of the triplet root, so the resulting path
# (".../qml") doesn't exist. Worse, the compiled Mu.* QML plugin DLLs (unlike
# their .qml/qmldir/qmltypes siblings) genuinely differ per config and vcpkg
# installs them into two separate trees -- "<triplet>/qml" (release) and
# "<triplet>/debug/qml" (debug) -- so a single variable can only ever resolve
# to one of them. A debug consumer that picks the release tree gets "uses
# incompatible Qt library (Cannot mix debug and release libraries.)" at
# runtime. Emit both variables, matching what frontend/CMakeLists.txt expects.
vcpkg_replace_string(
  "${CURRENT_PACKAGES_DIR}/share/mulib/MuLibConfig.cmake"
  "get_filename_component(_prefix \"\${_lib_dir}\" DIRECTORY)
set(MuLib_QML_IMPORT_DIR \"\${_prefix}/qml\")"
  "set(MuLib_QML_IMPORT_DIR_RELEASE \"\${_lib_dir}/qml\")
set(MuLib_QML_IMPORT_DIR_DEBUG \"\${_lib_dir}/debug/qml\")")

if("qml" IN_LIST FEATURES)
  foreach(_module IN ITEMS Material Model)
    if(NOT EXISTS "${CURRENT_PACKAGES_DIR}/qml/Mu/${_module}/qmldir")
      message(
        FATAL_ERROR
          "MuLib qml feature did not install qml/Mu/${_module}/qmldir. "
          "Check MuLib QML build and install rules at source ref "
          "8d587a98c01216d22af08ecd9619213b853c869b.")
    endif()
  endforeach()
endif()

# Guarded: the registry copy of this port ships no `usage` file, so the
# unguarded file(INSTALL) upstream is a hard build failure.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/usage")
  file(INSTALL "${CMAKE_CURRENT_LIST_DIR}/usage"
       DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}")
endif()

# Qt's multi-config QML install creates these configuration marker folders
# without files. Remove the folders themselves (not only a nonexistent lib
# child) so vcpkg's empty-directory post-build check is satisfied.
file(
  REMOVE_RECURSE
  "${CURRENT_PACKAGES_DIR}/debug/qml/Mu/Frontend/Debug"
  "${CURRENT_PACKAGES_DIR}/debug/qml/Mu/Geometry/Debug"
  "${CURRENT_PACKAGES_DIR}/debug/qml/Mu/Model/Debug"
  "${CURRENT_PACKAGES_DIR}/qml/Mu/Frontend/Release"
  "${CURRENT_PACKAGES_DIR}/qml/Mu/Geometry/Release"
  "${CURRENT_PACKAGES_DIR}/qml/Mu/Model/Release"
  "${CURRENT_PACKAGES_DIR}/debug/include")

if(EXISTS "${SOURCE_PATH}/LICENSE")
  vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
elseif(EXISTS "${SOURCE_PATH}/LICENSE.txt")
  vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE.txt")
else()
  file(WRITE "${CURRENT_PACKAGES_DIR}/share/${PORT}/copyright"
       "MuLib source repository; see the repository licensing terms.\n")
endif()
