# --- frontend (QtQuick/QML, Material style via mulib's Mu.Material) ---
# Only mu_core/mu_middleware are linked from mulib (kept for future use, e.g.
# Geometry2D/IBus utilities) -- rcu_qt_widgets (Widgets-only) isn't needed
# since the app no longer uses Qt Widgets.
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
    mu::core mu::middleware
)
target_include_directories(frontend PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)

# Mu.Material (MuApp/MuAppBar/MuCard/MuSparkline/...) is a file-based QML
# module built by mulib's own CMakeLists.txt (qt_add_qml_module(...
# NO_PLUGIN ...)) -- no plugin library to link, just a directory tree of
# .qml files + a hand-written qmldir under ${CMAKE_BINARY_DIR}/qml/Mu,
# resolved via a filesystem import path at runtime instead of a linked
# CMake target.
target_compile_definitions(frontend PRIVATE
    MULIB_QML_IMPORT_PATH="${CMAKE_BINARY_DIR}/qml")

# qt_add_qml_module(mu_material_qml_plugin ...) in mulib's own CMakeLists.txt
# auto-generates its own qmldir (with resource-relative, double-nested file
# paths like "qml/Mu/Material/MuApp.qml") as a side effect of processing
# QML_FILES; mulib's own mu_material_qml_files custom target is meant to
# overwrite that with a correct, flat-path qmldir afterwards, but there's no
# ordering guarantee between that target and this one (no CMake target links
# Mu.Material, it's consumed purely via filesystem import path), so the
# broken auto-generated version can still be what's on disk after frontend
# finishes building. Force the ordering, then re-apply mulib's own (already
# correct) fix script as the true last step.
add_dependencies(frontend mu_material_qml_plugin mu_material_qml_files)
add_custom_command(TARGET frontend POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        "-DOUTPUT_FILE=${CMAKE_BINARY_DIR}/qml/Mu/Material/qmldir"
        -P "${CMAKE_BINARY_DIR}/external/mulib/libs/mu_qml/WriteMuMaterialQmldir.cmake"
    COMMENT "Re-applying mulib's Mu.Material qmldir fix"
)

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
