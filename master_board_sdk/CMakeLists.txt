cmake_minimum_required(VERSION 3.8)
project(master_board_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

SET(MASTER_BOARD_SRC
    src/ESPNOW_manager.cpp
    src/ESPNOW_types.cpp
    src/ETHERNET_types.cpp
    src/Link_manager.cpp
    src/master_board_interface.cpp
    src/motor.cpp
    src/motor_driver.cpp
)

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${MASTER_BOARD_SRC}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} pthread)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

ament_export_targets(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)
ament_export_dependencies()

install(
  DIRECTORY include/
  DESTINATION include  
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
