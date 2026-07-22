# --- frontend (QtQuick/QML, Material style via mulib's Mu.Material) ---
# Linked from the Munera SDK:
#   mu::core    -- general utilities (kept for future use, e.g. Geometry2D)
#   mu::robot   -- RobotVisualModel, the list model MuMultiModelView renders
#   Ludus::Robot-- URDF import + forward kinematics behind RobotScene
# mu::cad is deliberately NOT linked: CadSceneGeometry is reached only through
# the Mu.Cad QML plugin at runtime, never from this target's C++.
# eCAL pub/sub moved from mu::middleware to the in-repo camera_tracking_middleware
# component, and the Widgets-only helpers aren't needed (no Qt Widgets here).
#
# Kept in its own subdirectory (rather than the top-level CMakeLists.txt)
# specifically so this target's SOURCE_DIR is frontend/, not the whole repo
# root -- qt6_import_qml_plugins()'s qmlimportscanner call uses SOURCE_DIR as
# its -rootPath and recursively scans it for .qml files, and the repo root
# also contains vcpkg/ (with buildtrees/ full of every vcpkg port's own
# extracted source, including qtdeclarative's own huge QML test suite).
qt_add_executable(frontend
    src/main.cpp
    src/Backend.cpp
    src/RobotScene.cpp
)
qt_add_qml_module(frontend
    URI CameraTracking
    VERSION 1.0
    QML_FILES
        qml/Main.qml
        qml/ViewportPanel.qml
        qml/PanelsPanel.qml
)
target_compile_features(frontend PRIVATE cxx_std_20)  # mulib's public headers require C++20
target_link_libraries(frontend PRIVATE
    camera_tracking_proto eCAL::core nlohmann_json::nlohmann_json
    Qt6::Quick Qt6::QuickControls2 Qt6::Qml
    mu::core mu::robot Ludus::Robot camera_tracking_middleware
)
target_include_directories(frontend PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)

# mulib assembles its whole QML import tree under ${CMAKE_BINARY_DIR}/qml:
# file-based Mu.Material (including MuMultiModelView/MuOrbitViewport) plus the
# *compiled* Mu.Cad and Mu.Robot plugins. MuMultiModelView does
# `import Mu.Cad 1.0` for CadSceneGeometry, so that plugin -- and its POST_BUILD
# copies of mu_cad.dll and Qt's private deps -- must exist before the
# windeployqt step below runs. MuLibAllQmlPlugins is mulib's own aggregate over
# all of it (including the mu_material_qml_files qmldir fix-up), so depending on
# it covers the whole tree in one edge. The engine then resolves the tree via
# the MULIB_QML_IMPORT_PATH import path added in main.cpp.
add_dependencies(frontend MuLibAllQmlPlugins)

# Which robot URDF the viewport loads at startup. Resolution order, first hit
# wins (the first two are explicit overrides, the third is the repo default):
#   1. CAMERA_TRACKING_ROBOT_URDF environment variable, read at runtime
#   2. -DCAMERA_TRACKING_ROBOT_URDF=<path> cache override at configure time
#   3. the flattened p2d2 cell shipped in Rendering/ (regenerate with
#      scripts/flatten_xacro.py if the xacro sources change)
# So a plain clone + configure + build renders the p2d2 cell with no setup.
set(CAMERA_TRACKING_ROBOT_URDF "" CACHE FILEPATH
    "Override the robot URDF shown in the viewport (empty = repo default Rendering/p2d2.urdf)")
if (CAMERA_TRACKING_ROBOT_URDF)
    set(_camera_tracking_robot_urdf "${CAMERA_TRACKING_ROBOT_URDF}")
else()
    set(_camera_tracking_robot_urdf "${CMAKE_SOURCE_DIR}/Rendering/p2d2.urdf")
endif()
target_compile_definitions(frontend PRIVATE
    CAMERA_TRACKING_ROBOT_URDF="${_camera_tracking_robot_urdf}")

# Mu.Material (MuApp/MuAppBar/MuCard/MuMultiModelView/...) is a file-based QML
# module built by mulib's own CMakeLists.txt (qt_add_qml_module(...
# NO_PLUGIN ...)) -- no plugin library to link, just a directory tree of
# .qml files + a hand-written qmldir under ${CMAKE_BINARY_DIR}/qml/Mu,
# resolved via a filesystem import path at runtime instead of a linked
# CMake target.
target_compile_definitions(frontend PRIVATE
    MULIB_QML_IMPORT_PATH="${CMAKE_BINARY_DIR}/qml")

# QML needs the whole QtQuick/Controls/Material (and Mu/Material) import
# tree deployed next to the exe at runtime (not just one plugin DLL,
# unlike the old Widgets build), so windeployqt replaces the old manual
# platforms/ dll copy. windeployqt.debug.bat/windeployqt.exe are the
# vcpkg-installed wrappers that point qtpaths at the matching
# debug/release Qt install.
if (WIN32)
    set(_camera_tracking_qt_tools_dir "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/tools/Qt6/bin")
    set(_camera_tracking_windeployqt
        "$<IF:$<CONFIG:Debug>,${_camera_tracking_qt_tools_dir}/windeployqt.debug.bat,${_camera_tracking_qt_tools_dir}/windeployqt.exe>")

    add_custom_command(TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "$<TARGET_FILE:frontend>: message: running windeployqt"
        COMMAND "${_camera_tracking_windeployqt}"
            --qmldir "${CMAKE_CURRENT_SOURCE_DIR}/qml"
            --qmldir "${CMAKE_BINARY_DIR}/qml"
            "$<TARGET_FILE:frontend>"
        COMMAND_EXPAND_LISTS
    )

    # vcpkg's Qt6 build bakes in "Qt6/qml" (matching its own
    # x64-windows/debug/Qt6/... install layout) as the hardcoded default QML
    # import path, not the plain "qml" that windeployqt actually deploys to
    # next to the exe -- ship our own qt.conf to override QmlImports back to
    # where the files really are (see doc.qt.io "Using qt.conf").
    add_custom_command(TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/qt.conf"
            "$<TARGET_FILE_DIR:frontend>/qt.conf"
    )
endif()
