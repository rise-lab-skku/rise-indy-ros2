cmake_minimum_required(VERSION 3.8)
project(indy_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_test test/test_urdf.py)
endif()

# Install launch files.
install(DIRECTORY
  launch robots meshes rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
