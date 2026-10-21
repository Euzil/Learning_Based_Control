function(ics_add_project_target)
    # check the input arguments
    if(ARGC EQUAL 0)
        message(STATUS "${YELLOW}WARNING: ics_add_project_target(): No DEPENDS targets for the project '${PROJECT_NAME}'!${NOCOLOR}")
    endif()
    
    set(PROJECT_TARGETS ${ARGN})
    
    message(STATUS "Add project target for '${PROJECT_NAME}'")
    
    # target for whole project
    add_custom_target(project_${PROJECT_NAME}
        DEPENDS ${PROJECT_TARGETS}
    )
    
endfunction()
