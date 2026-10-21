function(ics_create_find_module)
    set(options)
    set(oneValueArgs NAME INCLUDE_ROOT)
    set(multiValueArgs PATH_SUFFIXES LIBRARIES EXTERNAL_LIBRARIES FIND_PACKAGES INCLUDE_HINTS LIBRARY_HINTS)
    
    ics_parse_arguments(FIND_MODULE "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    message(STATUS "FIND_MODULE_NAME                = " ${FIND_MODULE_NAME})
    message(STATUS "FIND_MODULE_INCLUDE_ROOT        = " ${FIND_MODULE_INCLUDE_ROOT})
    message(STATUS "FIND_MODULE_LIBRARIES           = " ${FIND_MODULE_LIBRARIES})
    message(STATUS "FIND_MODULE_PATH_SUFFIXES       = " ${FIND_MODULE_PATH_SUFFIXES})
    message(STATUS "FIND_MODULE_EXTERNAL_LIBRARIES  = " ${FIND_MODULE_EXTERNAL_LIBRARIES})
    message(STATUS "FIND_MODULE_FIND_PACKAGES       = " ${FIND_MODULE_FIND_PACKAGES})
    message(STATUS "FIND_MODULE_INCLUDE_HINTS       = " ${FIND_MODULE_INCLUDE_HINTS})
    message(STATUS "FIND_MODULE_LIBRARY_HINTS       = " ${FIND_MODULE_LIBRARY_HINTS})

    if(NOT FIND_MODULE_NAME)
        message(FATAL_ERROR "'NAME' for find module not set!")
    endif()
    
    if(NOT FIND_MODULE_INCLUDE_ROOT)
        message(FATAL_ERROR "'INCLUDE_ROOT' for find module not set!")
    endif()
    
    if(NOT FIND_MODULE_LIBRARIES)
        message(FATAL_ERROR "'FIND_MODULE_LIBRARIES' for find module not set!")
    endif()
    
    # create find module
    configure_file(
        "${ICS_TOOLS_DIR}/ics_create_find_module.cmake.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Find${FIND_MODULE_NAME}.cmake"
        IMMEDIATE @ONLY)
endfunction()
