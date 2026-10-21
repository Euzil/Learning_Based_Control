function(ics_create_local_shell_script)
    set(options)
    set(oneValueArgs)
    set(multiValueArgs TARGETS LOAD_LIBRARY_PATHS)
    
    ics_parse_arguments(SHELL_SCRIPT "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    #message(STATUS ${ARGN})
    message(STATUS "SHELL_SCRIPT_TARGETS            = " ${SHELL_SCRIPT_TARGETS})
    message(STATUS "SHELL_SCRIPT_LOAD_LIBRARY_PATHS = " ${SHELL_SCRIPT_LOAD_LIBRARY_PATHS})

    if(NOT SHELL_SCRIPT_TARGETS)
        message(FATAL_ERROR "'TARGETS' for local scripts not set!")
    endif()
    
    # make load library paths relative
    set(LOAD_LIBRARY_PATHS)
    foreach(path ${SHELL_SCRIPT_LOAD_LIBRARY_PATHS})
        file(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${path})
        set(LOAD_LIBRARY_PATHS ${LOAD_LIBRARY_PATHS} ${rel})
    endforeach()
    
    # create scripts
    foreach(target ${SHELL_SCRIPT_TARGETS})
        message(STATUS "target = " ${target})
        
        # test if target is an executable
        get_target_property(target_type ${target} TYPE)
        if(NOT target_type STREQUAL "EXECUTABLE")
            continue()
        endif()
        
        file(RELATIVE_PATH rel ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_BINARY_DIR})
        
        # set variable for script config file
        set(EXE_NAME_PATH "${rel}/${target}")

        # create shell script
        configure_file(
            "${ICS_TOOLS_DIR}/ics_create_local_shell_script.sh.in"
            "${PROJECT_BINARY_DIR}/${target}.sh"
            IMMEDIATE @ONLY)
            
        file(COPY "${PROJECT_BINARY_DIR}/${target}.sh"
            DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
            FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)    
        
    endforeach()
    

endfunction()
