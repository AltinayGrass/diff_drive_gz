cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_description)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install project urdf files
install(
  DIRECTORY
    urdf
    meshes
    models
  DESTINATION 
    share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
