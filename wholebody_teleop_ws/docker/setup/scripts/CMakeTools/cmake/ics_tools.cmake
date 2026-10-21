set(ICS_TOOLS_DIR ${CMAKE_CURRENT_LIST_DIR})

include(${ICS_TOOLS_DIR}/print_target_properties.cmake)

include(${ICS_TOOLS_DIR}/ics_colors.cmake)
include(${ICS_TOOLS_DIR}/ics_ros_debgen.cmake)


include(${ICS_TOOLS_DIR}/ics_add_install_local.cmake)
include(${ICS_TOOLS_DIR}/ics_add_project_target.cmake)
include(${ICS_TOOLS_DIR}/ics_add_subproject.cmake)
include(${ICS_TOOLS_DIR}/ics_add_uninstall.cmake)

include(${ICS_TOOLS_DIR}/ics_parse_arguments.cmake)
include(${ICS_TOOLS_DIR}/ics_create_find_module.cmake)
include(${ICS_TOOLS_DIR}/ics_create_local_shell_script.cmake)


include(${ICS_TOOLS_DIR}/ics_use_strict_warnings.cmake)











