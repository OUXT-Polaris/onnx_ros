cmake_minimum_required(VERSION 3.8)
project(onnx_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(detic_component SHARED
  src/detic_component.cpp
)
ament_auto_add_executable(detic_node
  src/detic_component.cpp
  src/detic_node.cpp
)

file(DOWNLOAD
https://storage.googleapis.com/ailia-models/detic/Detic_C2_SwinB_896_4x_IN-21K+COCO_lvis_op16.onnx
  ${CMAKE_CURRENT_BINARY_DIR}/models/Detic_C2_SwinB_896_4x_IN-21K+COCO_lvis_op16.onnx
  SHOW_PROGRESS
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/models/Detic_C2_SwinB_896_4x_IN-21K+COCO_lvis_op16.onnx
  DESTINATION share/${PROJECT_NAME}/models)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
