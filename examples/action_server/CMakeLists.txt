cmake_minimum_required(VERSION 3.16.3)
project(example_action)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    repo_cmake
    example_interfaces
    rclcpp
    rclcpp_action
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#########################################
# Example class that uses Action Server #
#########################################

add_library(${PROJECT_NAME}
  SHARED
    src/fibber.cpp
		src/rcl_handle.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    repo_cmake::project_warnings
    repo_cmake::project_options
)

#########################################
# Example node that uses class          #
#########################################

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_20)
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME}_node
	PRIVATE
	${PROJECT_NAME}
	repo_cmake::project_options
	repo_cmake::project_warnings
	)


install(
  DIRECTORY
    include
  DESTINATION
    share/${PROJECT_NAME}
	)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT
    ${PROJECT_NAME}_Targets
  ARCHIVE DESTINATION
    lib
  LIBRARY DESTINATION
    lib
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
	)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
	)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE) # Does not work with ...
  set(ament_cmake_copyright_FOUND TRUE) # Restore once we are back on Rolling
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-format
  set(ament_cmake_flake8_FOUND TRUE) # Conflicts with black
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()

  # file(GLOB_RECURSE test_srcs "test/*.cpp")

  # target_link_libraries(test_${PROJECT_NAME}
  #     PRIVATE
  #     ${PROJECT_NAME}
  #     repo_cmake::project_options
  #     repo_cmake::project_warnings
  # )
endif()

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_Targets)

ament_package()
