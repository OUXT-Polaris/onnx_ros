#ifndef ONNX_ROS__VISIBILITY_CONTROL_H_
#define ONNX_ROS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define ONNX_ROS_EXPORT __attribute__((dllexport))
#define ONNX_ROS_IMPORT __attribute__((dllimport))
#else
#define ONNX_ROS_EXPORT __declspec(dllexport)
#define ONNX_ROS_IMPORT __declspec(dllimport)
#endif
#ifdef ONNX_ROS_BUILDING_LIBRARY
#define ONNX_ROS_PUBLIC ONNX_ROS_EXPORT
#else
#define ONNX_ROS_PUBLIC ONNX_ROS_IMPORT
#endif
#define ONNX_ROS_PUBLIC_TYPE ONNX_ROS_PUBLIC
#define ONNX_ROS_LOCAL
#else
#define ONNX_ROS_EXPORT __attribute__((visibility("default")))
#define ONNX_ROS_IMPORT
#if __GNUC__ >= 4
#define ONNX_ROS_PUBLIC __attribute__((visibility("default")))
#define ONNX_ROS_LOCAL __attribute__((visibility("hidden")))
#else
#define ONNX_ROS_PUBLIC
#define ONNX_ROS_LOCAL
#endif
#define ONNX_ROS_PUBLIC_TYPE
#endif

#endif  // ONNX_ROS__VISIBILITY_CONTROL_H_
