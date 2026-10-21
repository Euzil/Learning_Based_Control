function(ics_add_subproject)   
    #message(STATUS "Input args: ${ARGN}")
    #message(STATUS "Num of input args: ${ARGC}")
    
    # let the sub projects know that they have a parent project
    set(HAS_PARENT_PROJECT TRUE)

    # check the input arguments
    if(NOT ARGC EQUAL 1)
        message(FATAL_ERROR "ics_add_subproject(): Invalid number of input args!")
    endif()
    
    get_filename_component(subProject ${ARGN} NAME)
    #set(subProjectDir ${ARGN})
    
    file(RELATIVE_PATH subProjectDir ${CMAKE_CURRENT_SOURCE_DIR} ${ARGN})
    
    message(STATUS "${BGREEN}+++${NOCOLOR} Add sub project: '${BBLUE}${subProject}${NOCOLOR}' located in '${subProjectDir}'.")
    
    if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${subProjectDir}")
        message(FATAL_ERROR "ics_add_subproject(): The sub project '${subProject}' doesn't exist!")
    endif()
    
    # add the find module of the sub project
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/${subProjectDir}/cmake" PARENT_SCOPE)
    
    # add the sub project
    add_subdirectory(${subProjectDir})    
    
    # check if the project target exists
    set(PROJECT_TARGET "project_${subProject}")
    if(TARGET ${PROJECT_TARGET})
        list(APPEND PROJECT_TARGET_LIST ${PROJECT_TARGET})
        list(REMOVE_DUPLICATES PROJECT_TARGET_LIST)
        set(PROJECT_TARGET_LIST ${PROJECT_TARGET_LIST} PARENT_SCOPE)
    else()
        message(STATUS "${YELLOW}WARNING: ics_add_subproject(): The project '${subProject}' doesn't have a project target '${PROJECT_TARGET}'!${NOCOLOR}")
    endif()
    
    set(installDir "${CMAKE_CURRENT_SOURCE_DIR}/${subProjectDir}/install")
    
    #message(STATUS "${subProject}: CMAKE_INSTALL_RPATH_USE_LINK_PATH=${CMAKE_INSTALL_RPATH_USE_LINK_PATH}")
    
    # install local for sub project
    add_custom_target(install_local_${subProject} ALL
        DEPENDS ${PROJECT_TARGET}
        COMMAND rm -rf ${installDir}
        COMMAND "${CMAKE_COMMAND}" -DCMAKE_INSTALL_PREFIX=${installDir}
                -P "${CMAKE_CURRENT_BINARY_DIR}/${subProjectDir}/cmake_install.cmake"
    )
    
endfunction()