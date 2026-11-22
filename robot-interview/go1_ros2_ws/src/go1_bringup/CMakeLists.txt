cmake_minimum_required(VERSION 3.8)
project(go1_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(GO1_MCP_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../phanthymotus-driver/interview/go1_mcp"
  CACHE PATH "Location of the Phanthy Motus Go1 MCP driver source")
ament_python_install_package(go1_mcp PACKAGE_DIR "${GO1_MCP_SOURCE_DIR}/go1_mcp")
install(PROGRAMS "${GO1_MCP_SOURCE_DIR}/go1_mcp_node" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_package()
