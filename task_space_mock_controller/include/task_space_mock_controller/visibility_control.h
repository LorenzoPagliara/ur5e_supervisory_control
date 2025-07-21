/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   visibility_control.h
 * Author:  Davide Risi
 * Org.:    UNISA
 * Date:    Dec 3, 2024
 *
 * This module contains the visibility control macros for the
 * reference_generator package.
 * -------------------------------------------------------------------
 */

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef TASK_SPACE_MOCK_CONTROLLER__VISIBILITY_CONTROL_H_
#define TASK_SPACE_MOCK_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TASK_SPACE_MOCK_CONTROLLER_EXPORT __attribute__((dllexport))
#define TASK_SPACE_MOCK_CONTROLLER_IMPORT __attribute__((dllimport))
#else
#define TASK_SPACE_MOCK_CONTROLLER_EXPORT __declspec(dllexport)
#define TASK_SPACE_MOCK_CONTROLLER_IMPORT __declspec(dllimport)
#endif
#ifdef TASK_SPACE_MOCK_CONTROLLER_BUILDING_DLL
#define TASK_SPACE_MOCK_CONTROLLER_PUBLIC TASK_SPACE_MOCK_CONTROLLER_EXPORT
#else
#define TASK_SPACE_MOCK_CONTROLLER_PUBLIC TASK_SPACE_MOCK_CONTROLLER_IMPORT
#endif
#define TASK_SPACE_MOCK_CONTROLLER_PUBLIC_TYPE TASK_SPACE_MOCK_CONTROLLER_PUBLIC
#define TASK_SPACE_MOCK_CONTROLLER_LOCAL
#else
#define TASK_SPACE_MOCK_CONTROLLER_EXPORT __attribute__((visibility("default")))
#define TASK_SPACE_MOCK_CONTROLLER_IMPORT
#if __GNUC__ >= 4
#define TASK_SPACE_MOCK_CONTROLLER_PUBLIC __attribute__((visibility("default")))
#define TASK_SPACE_MOCK_CONTROLLER_LOCAL __attribute__((visibility("hidden")))
#else
#define TASK_SPACE_MOCK_CONTROLLER_PUBLIC
#define TASK_SPACE_MOCK_CONTROLLER_LOCAL
#endif
#define TASK_SPACE_MOCK_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // TASK_SPACE_MOCK_CONTROLLER__VISIBILITY_CONTROL_H_
