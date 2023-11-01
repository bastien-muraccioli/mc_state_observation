# Copyright 2020 CNRS-AIST JRL
#
# Try to find ROS and some required ROS packages
#
# Defines mc_state_observation::ROS if everything is found

function(mc_state_observation_ros2_dependency PKG)
  find_package(${PKG} REQUIRED)
  target_link_libraries(mc_rtc_3rd_party::ROS INTERFACE ${PKG}::${PKG})
endfunction()

macro(mc_state_observation_set_result result)
  set(ROSCPP_FOUND ${result})
  set(mc_state_observation_ROS_FOUND ${result})
endmacro()

if(NOT TARGET mc_state_observation::ROS)
  if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" EQUAL "2")
    list(APPEND CMAKE_PREFIX_PATH $ENV{AMENT_PREFIX_PATH}
         $ENV{COLCON_PREFIX_PATH})
    set(AMENT_CMAKE_UNINSTALL_TARGET
        OFF
        CACHE BOOL "" FORCE)
    find_package(rclcpp QUIET)
    if(NOT TARGET rclcpp::rclcpp)
      mc_state_observation_set_result(False)
      return()
    endif()
    add_library(mc_state_observation::ROS INTERFACE IMPORTED)
    target_link_libraries(mc_state_observation::ROS INTERFACE rclcpp::rclcpp)
    mc_state_observation_ros2_dependency(tf2)
    mc_state_observation_ros2_dependency(tf2_eigen)
    target_compile_definitions(mc_state_observation::ROS
                               INTERFACE MC_STATE_OBSERVATION_ROS_IS_ROS2)
    mc_state_observation_set_result(True)
    return()
  endif()
  # ROS1 path
  if(NOT COMMAND pkg_check_modules)
    find_package(PkgConfig)
  endif()
  pkg_check_modules(MC_STATE_OBSERVATION_roscpp QUIET roscpp)
  if(${MC_STATE_OBSERVATION_roscpp_FOUND})
    mc_state_observation_set_result(True)
    set(MC_STATE_OBSERVATION_ROS_DEPENDENCIES roscpp tf2 tf2_eigen)
    foreach(DEP ${MC_STATE_OBSERVATION_ROS_DEPENDENCIES})
      pkg_check_modules(MC_STATE_OBSERVATION_${DEP} REQUIRED ${DEP})
      list(APPEND MC_STATE_OBSERVATION_ROS_LIBRARIES
           ${MC_STATE_OBSERVATION_${DEP}_LIBRARIES})
      list(APPEND MC_STATE_OBSERVATION_ROS_LIBRARY_DIRS
           ${MC_STATE_OBSERVATION_${DEP}_LIBRARY_DIRS})
      list(APPEND MC_STATE_OBSERVATION_ROS_INCLUDE_DIRS
           ${MC_STATE_OBSERVATION_${DEP}_INCLUDE_DIRS})
      foreach(FLAG ${MC_STATE_OBSERVATION_${DEP}_LDFLAGS})
        if(IS_ABSOLUTE ${FLAG})
          list(APPEND MC_STATE_OBSERVATION_ROS_FULL_LIBRARIES ${FLAG})
        endif()
      endforeach()
    endforeach()
    list(REMOVE_DUPLICATES MC_STATE_OBSERVATION_ROS_LIBRARIES)
    list(REMOVE_DUPLICATES MC_STATE_OBSERVATION_ROS_LIBRARY_DIRS)
    list(REMOVE_DUPLICATES MC_STATE_OBSERVATION_ROS_INCLUDE_DIRS)
    foreach(LIB ${MC_STATE_OBSERVATION_ROS_LIBRARIES})
      string(SUBSTRING "${LIB}" 0 1 LIB_STARTS_WITH_COLUMN)
      if(${LIB_STARTS_WITH_COLUMN} STREQUAL ":")
        string(SUBSTRING "${LIB}" 1 -1 LIB)
      endif()
      if(IS_ABSOLUTE ${LIB})
        list(APPEND MC_STATE_OBSERVATION_ROS_FULL_LIBRARIES ${LIB})
      else()
        find_library(${LIB}_FULL_PATH NAME ${LIB}
                     HINTS ${MC_STATE_OBSERVATION_ROS_LIBRARY_DIRS})
        list(APPEND MC_STATE_OBSERVATION_ROS_FULL_LIBRARIES ${${LIB}_FULL_PATH})
      endif()
    endforeach()
    list(REMOVE_DUPLICATES MC_STATE_OBSERVATION_ROS_FULL_LIBRARIES)
    add_library(mc_state_observation::ROS INTERFACE IMPORTED)
    set_target_properties(
      mc_state_observation::ROS
      PROPERTIES INTERFACE_LINK_LIBRARIES
                 "${MC_STATE_OBSERVATION_ROS_FULL_LIBRARIES}"
                 INTERFACE_INCLUDE_DIRECTORIES
                 "${MC_STATE_OBSERVATION_ROS_INCLUDE_DIRS}")
    message(
      "-- Found ROS libraries: ${MC_STATE_OBSERVATION_ROS_FULL_LIBRARIES}")
    message(
      "-- Found ROS include directories: ${MC_STATE_OBSERVATION_ROS_INCLUDE_DIRS}"
    )
  else()
    mc_state_observation_set_result(False)
  endif()
endif()
