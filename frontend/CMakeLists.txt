# --- frontend (QtQuick/QML, Material style via mulib's Mu.Material) ---
# Linked from the Munera SDK (installed vcpkg-registry ports):
#   MuLib::Core  -- general utilities (kept for future use, e.g. Geometry2D)
#   MuLib::Robot -- RobotVisualModel, the list model MuMultiModelView renders
#   Ludus::Robot -- URDF import + KDL forward kinematics + robot scene builder

qt_add_executable(frontend
    src/main.cpp
    src/Backend.cpp
    src/RobotScene.cpp
)
# Theme is the single place the Siemens Energy palette is written down; it has
# to be registered as a singleton or `import CameraTracking` yields a type, not
# an instance. This must precede qt_add_qml_module, which reads the property.
set_source_files_properties(qml/Theme.qml PROPERTIES QT_QML_SINGLETON_TYPE TRUE)
qt_add_qml_module(frontend
    URI CameraTracking
    VERSION 1.0
    QML_FILES
        qml/Main.qml
        qml/Theme.qml
        qml/ThemedCard.qml
        qml/StatusChip.qml
        qml/ViewportPanel.qml
        # One bottom bar per workflow screen. The viewport is shared across all
        # three (Main.qml owns the single instance), so these are the only
        # per-screen QML -- which is also why PanelsPanel.qml is gone: the side
        # card it drew was replaced by TrackingBar.
        qml/HomeBar.qml
        qml/RotorPlacementBar.qml
        qml/TrackingBar.qml
)
target_compile_features(frontend PRIVATE cxx_std_20)  # mulib's public headers require C++20
target_link_libraries(frontend PRIVATE
    camera_tracking_proto eCAL::core nlohmann_json::nlohmann_json
    Qt6::Quick Qt6::QuickControls2 Qt6::Qml
    MuLib::Core MuLib::Robot Ludus::Robot camera_tracking_middleware
    Eigen3::Eigen
)
target_include_directories(frontend PRIVATE ${CMAKE_SOURCE_DIR}/nodes/common)
# Local-source mode exposes this aggregate QML-plugin target; the installed port
# has the plugins prebuilt, so it won't exist there. Guard so both modes work.
if (TARGET MuLibAllQmlPlugins)
    add_dependencies(frontend MuLibAllQmlPlugins)
endif()
# The URDF, rotor mesh and tracked arm used to be compile-time definitions.
# They now live in the scene manifest (config/scene.json + config/objects/*.json)
# so that changing what is rendered is a config edit rather than a rebuild, and
# so the frontend and backend cannot disagree about what the scene contains.
# mulib's QML tree now ships inside the installed port. find_package(MuLib)
# defines these import roots; pick the one matching the build config. This
# split is not cosmetic: vcpkg installs the compiled Mu.* QML plugin DLLs
# into two separate trees (release under the triplet root, debug under
# triplet/debug/), and Qt refuses to load a release plugin into a debug
# process ("Cannot mix debug and release libraries").
set(_mulib_qml_import_dir
    "$<IF:$<CONFIG:Debug>,${MuLib_QML_IMPORT_DIR_DEBUG},${MuLib_QML_IMPORT_DIR_RELEASE}>")
target_compile_definitions(frontend PRIVATE
    MULIB_QML_IMPORT_PATH="${_mulib_qml_import_dir}")
if (WIN32)
    set(_camera_tracking_qt_tools_dir "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/tools/Qt6/bin")
    set(_camera_tracking_windeployqt
        "$<IF:$<CONFIG:Debug>,${_camera_tracking_qt_tools_dir}/windeployqt.debug.bat,${_camera_tracking_qt_tools_dir}/windeployqt.exe>")

    add_custom_command(TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "$<TARGET_FILE:frontend>: message: running windeployqt"
        COMMAND "${_camera_tracking_windeployqt}"
            --qmldir "${CMAKE_CURRENT_SOURCE_DIR}/qml"
            --qmldir "${_mulib_qml_import_dir}"
            "$<TARGET_FILE:frontend>"
        COMMAND_EXPAND_LISTS
    )
    add_custom_command(TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_CURRENT_SOURCE_DIR}/qt.conf"
            "$<TARGET_FILE_DIR:frontend>/qt.conf"
    )

    if(NOT DEFINED Z_VCPKG_EXECUTABLE)
        message(FATAL_ERROR
            "Z_VCPKG_EXECUTABLE is not set -- expected to be defined by "
            "vcpkg's own toolchain file (vcpkg/scripts/buildsystems/vcpkg.cmake). "
            "frontend/cmake/DeployQmlPluginDeps.cmake needs it to resolve "
            "runtime dependencies of dynamically-loaded QML plugins.")
    endif()
    set(_camera_tracking_installed_bin_dir
        "$<IF:$<CONFIG:Debug>,${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/debug/bin,${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin>")
    # Only these two plugin trees are known to pull in dependencies outside
    # what windeployqt/VCPKG_APPLOCAL_DEPS already resolve at Debug/ root:
    #   - qml/QtQuick3D -- the one Qt QML module frontend links (of Quick,
    #     QuickControls2, Qml, Quick3D) with genuine third-party (non-Qt)
    #     dependencies, e.g. meshoptimizer.dll.
    #   - mulib's Mu.* plugins -- loaded from the vcpkg install tree via an
    #     absolute path (MULIB_QML_IMPORT_PATH) rather than a windeployqt
    #     copy, so their own internal dependencies (e.g. mu_cad.dll) are
    #     invisible to every automatic mechanism above.
    # See frontend/cmake/DeployQmlPluginDeps.cmake for why this is scoped
    # to specific directories rather than the whole deployed qml tree.
    add_custom_command(TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "$<TARGET_FILE:frontend>: message: resolving QML plugin runtime deps"
        COMMAND ${CMAKE_COMMAND}
            -DVCPKG_EXE=${Z_VCPKG_EXECUTABLE}
            "-DQML_PLUGIN_DIR=$<TARGET_FILE_DIR:frontend>/qml/QtQuick3D"
            -DINSTALLED_BIN_DIR=${_camera_tracking_installed_bin_dir}
            "-DPRIMARY_OUTPUT_DIR=$<TARGET_FILE_DIR:frontend>"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DeployQmlPluginDeps.cmake"
        COMMAND ${CMAKE_COMMAND}
            -DVCPKG_EXE=${Z_VCPKG_EXECUTABLE}
            "-DQML_PLUGIN_DIR=${_mulib_qml_import_dir}"
            -DINSTALLED_BIN_DIR=${_camera_tracking_installed_bin_dir}
            "-DPRIMARY_OUTPUT_DIR=$<TARGET_FILE_DIR:frontend>"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DeployQmlPluginDeps.cmake"
    )
endif()
