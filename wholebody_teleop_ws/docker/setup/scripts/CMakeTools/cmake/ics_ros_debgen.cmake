function(ics_ros_debgen) 
   if(DEB_BUILD)
      MESSAGE(" ${YELLOW}DEB_BUILD: The standard DESTINATIONS will be adapted to build deb packages!${NOCOLOR}")
      ###########################################################################
      # NOTE: We need to modify this variables to be compliant with the debtools
      # For a plain ROS environment comment these lines
      set (CATKIN_PACKAGE_LIB_DESTINATION "lib/${PROJECT_NAME}" PARENT_SCOPE)
      set (CATKIN_PACKAGE_BIN_DESTINATION "bin/${PROJECT_NAME}" PARENT_SCOPE)
      set (CATKIN_PACKAGE_SHARE_DESTINATION "share/${PROJECT_NAME}" PARENT_SCOPE)
      set (CATKIN_PACKAGE_INCLUDE_DESTINATION "include/${PROJECT_NAME}" PARENT_SCOPE)
      ###########################################################################
   else()
      MESSAGE("${GREEN}Using the standard catkin DESTINATIONS${NOCOLOR}")
   endif()
endfunction()