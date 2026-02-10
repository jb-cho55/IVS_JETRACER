#ifndef JETRACER_ROS_WRAP__VISIBILITY_CONTROL_H_
#define JETRACER_ROS_WRAP__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define JETRACER_ROS_WRAP_EXPORT __attribute__ ((dllexport))
    #define JETRACER_ROS_WRAP_IMPORT __attribute__ ((dllimport))
  #else
    #define JETRACER_ROS_WRAP_EXPORT __declspec(dllexport)
    #define JETRACER_ROS_WRAP_IMPORT __declspec(dllimport)
  #endif
  #ifdef JETRACER_ROS_WRAP_BUILDING_LIBRARY
    #define JETRACER_ROS_WRAP_PUBLIC JETRACER_ROS_WRAP_EXPORT
  #else
    #define JETRACER_ROS_WRAP_PUBLIC JETRACER_ROS_WRAP_IMPORT
  #endif
  #define JETRACER_ROS_WRAP_PUBLIC_TYPE JETRACER_ROS_WRAP_PUBLIC
  #define JETRACER_ROS_WRAP_LOCAL
#else
  #define JETRACER_ROS_WRAP_EXPORT __attribute__ ((visibility("default")))
  #define JETRACER_ROS_WRAP_IMPORT
  #if __GNUC__ >= 4
    #define JETRACER_ROS_WRAP_PUBLIC __attribute__ ((visibility("default")))
    #define JETRACER_ROS_WRAP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define JETRACER_ROS_WRAP_PUBLIC
    #define JETRACER_ROS_WRAP_LOCAL
  #endif
  #define JETRACER_ROS_WRAP_PUBLIC_TYPE
#endif
#endif  // JETRACER_ROS_WRAP__VISIBILITY_CONTROL_H_
// Generated 10-Feb-2026 18:00:35
// Copyright 2019-2020 The MathWorks, Inc.
