cmake_minimum_required(VERSION 3.8)
project(aictrain_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

nav2_package()

include_directories(include)

set(DEPENDS nav2_common nav2_core pluginlib)

add_library(${PROJECT_NAME} SHARED src/aictrain.cpp src/aictrain_planner.cpp)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDS})

pluginlib_export_plugin_description_file(nav2_core aictrain_planner.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/)

install(FILES aictrain_planner.xml DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${DEPENDS})
ament_package()
