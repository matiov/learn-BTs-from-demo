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

#===================================================================================================
# Installation
#===================================================================================================
include(GNUInstallDirs)

install(
  DIRECTORY
    "launch"
    "meshes"
    "properties"
    "urdf"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}"
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
ament_package()
