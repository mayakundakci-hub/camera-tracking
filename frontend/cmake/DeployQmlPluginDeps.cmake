
if(NOT DEFINED VCPKG_EXE OR NOT DEFINED QML_PLUGIN_DIR OR NOT DEFINED INSTALLED_BIN_DIR
   OR NOT DEFINED PRIMARY_OUTPUT_DIR)
  message(FATAL_ERROR
    "DeployQmlPluginDeps.cmake requires -DVCPKG_EXE=, -DQML_PLUGIN_DIR=, "
    "-DINSTALLED_BIN_DIR=, -DPRIMARY_OUTPUT_DIR=")
endif()

file(GLOB_RECURSE _dlls "${QML_PLUGIN_DIR}/*.dll")

foreach(_dll IN LISTS _dlls)
  execute_process(
    COMMAND "${VCPKG_EXE}" z-applocal
            "--target-binary=${_dll}"
            "--installed-bin-dir=${INSTALLED_BIN_DIR}"
    RESULT_VARIABLE _result
  )
  if(NOT _result EQUAL 0)
    message(WARNING "vcpkg z-applocal failed for ${_dll} (exit ${_result})")
  endif()
endforeach()

file(GLOB_RECURSE _resolved_dlls "${QML_PLUGIN_DIR}/*.dll")
foreach(_dll IN LISTS _resolved_dlls)
  get_filename_component(_name "${_dll}" NAME)
  file(COPY_FILE "${_dll}" "${PRIMARY_OUTPUT_DIR}/${_name}" ONLY_IF_DIFFERENT)
endforeach()
