cmake_minimum_required(VERSION 2.6)

# define project name
project(CMakeTools NONE)

# install
install(DIRECTORY cmake/
    DESTINATION share/cmake/CMakeTools
    PATTERN "*~" EXCLUDE
    PATTERN "*.swp" EXCLUDE
    PATTERN "*.cmake"
)


# install local target
# set(installDir ${CMAKE_CURRENT_SOURCE_DIR}/install)
# add_custom_target(install_local
#     DEPENDS
#     COMMAND rm -rf ${installDir}
#     COMMAND "${CMAKE_COMMAND}" -DCMAKE_INSTALL_PREFIX=${installDir}
#             -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake"
# )

# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
