function(ics_add_install_local)

    # check the input arguments
#     if(NOT ARGC EQUAL 1)
#         message(FATAL_ERROR "ics_add_install_local(): Invalid number of input args!")
#     endif()

#     if(ARGC EQUAL 0 AND NOT DEFINED PROJECT_TARGET_LIST)
#         message(FATAL_ERROR "ics_add_install_local(): You need to provide targets!")
#     endif()

    if(ARGC EQUAL 0 AND NOT DEFINED PROJECT_TARGET_LIST)
        message(STATUS "${YELLOW}WARNING: ics_add_install_local(): No DEPENDS targets for the project '${PROJECT_NAME}'!${NOCOLOR}")
    endif()


    if(ARGC EQUAL 0 AND DEFINED PROJECT_TARGET_LIST)
        set(PROJECT_TARGETS ${PROJECT_TARGET_LIST})
    endif()
    
    if(NOT ARGC EQUAL 0)
        set(PROJECT_TARGETS ${ARGN})
    endif()


    foreach(target ${PROJECT_TARGETS})
        message(STATUS "Target: ${target}")
    endforeach()

    set(installDir ${CMAKE_CURRENT_SOURCE_DIR}/install)
    
    add_custom_target(install_local ALL
        DEPENDS ${PROJECT_TARGETS}
        COMMAND rm -rf ${installDir}
        COMMAND "${CMAKE_COMMAND}" -DCMAKE_INSTALL_PREFIX=${installDir}
                -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake"
    )
endfunction()