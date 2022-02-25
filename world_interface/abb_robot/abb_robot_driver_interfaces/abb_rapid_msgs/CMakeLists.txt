cmake_minimum_required(VERSION 3.16.3)

####################################################################################################
# Metadata
####################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

####################################################################################################
# CMake project
####################################################################################################
project(${extracted_name} VERSION ${extracted_version})

#===================================================================================================
# Dependencies
#===================================================================================================
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#===================================================================================================
# Settings
#===================================================================================================
# [Note]: Workaround (2021-10-25, using Foxy on Ubuntu 20.04) for issue with
#         custom ROS interface defintions not working with static libraries.
if(DEFINED BUILD_SHARED_LIBS)
  if(NOT BUILD_SHARED_LIBS)
    message(WARNING "Overriding to building shared libraries")
    set(BUILD_SHARED_LIBS ON)
  endif()
endif()

#===================================================================================================
# Generate ROS interfaces
#===================================================================================================
set(
  definition_files
    # Message defintions.
    "msg/ConfData.msg"
    "msg/ExtJoint.msg"
    "msg/JointTarget.msg"
    "msg/LoadData.msg"
    "msg/Orient.msg"
    "msg/Pos.msg"
    "msg/Pose.msg"
    "msg/RobJoint.msg"
    "msg/RobTarget.msg"
    "msg/ToolData.msg"
    "msg/WObjData.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${definition_files}
  DEPENDENCIES
    builtin_interfaces
  ADD_LINTER_TESTS
)

#===================================================================================================
# Testing
#===================================================================================================
if(BUILD_TESTING)
  #-------------------------------------------------------------------------------------------------
  # Linter tests
  #-------------------------------------------------------------------------------------------------
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
  )

  ament_lint_auto_find_test_dependencies()
endif()

#===================================================================================================
# Finalize ament package
#===================================================================================================
ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
