# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.5)
project(onnxruntime_vendor)

find_package(ament_cmake REQUIRED)

option(CUDA_SUPPORT "use CUDA support onnxruntime library" OFF)

if(CUDA_SUPPORT)
    message("Building with Cuda support")
    set(PACKAGE_URL "https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime.Gpu/1.16.1")
    set(PACKAGE_SHA512 "d2312b40aedd37967d2d7a4c538da39b0c1852f291dbde6b88048f66a9ddd73fddca95ce46cbb0eb441367148bb2b5ee34f443cd1b8059caceb60c09ba96a25a")
else()
    message("Building without Cuda support")
    set(PACKAGE_URL "https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/1.16.1")
    set(PACKAGE_SHA512 "95ab206b0690a3b94b7359492d39a5b16fea90fe9877ffe1d975f4e724c9a4819cd786428998f0e68a604c4ed277044cde38ce1de9cfad2c1aa1a150309b5eb0")
endif()

file(DOWNLOAD
    ${PACKAGE_URL}
    ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime.nuget
    EXPECTED_HASH SHA512=${PACKAGE_SHA512}
    SHOW_PROGRESS
)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime")

add_custom_target(ExtractOnnxruntime ALL
    ${CMAKE_COMMAND} -E
    tar xvzf "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime.nuget"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/onnxruntime"
)

install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/build/native/include/
    DESTINATION include
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/runtimes/linux-x64/native/libonnxruntime.so
    DESTINATION lib
    RENAME libonnxruntime.so.1.16.1
)

ament_export_libraries(onnxruntime)

# install(
#     FILES
#         ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/LICENSE.txt
#         ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime/ThirdPartyNotices.txt
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package(
  CONFIG_EXTRAS "cmake/onnxruntime_vendor-extras.cmake"
)
