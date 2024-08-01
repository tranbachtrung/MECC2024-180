/*
 * This file has been automatically generated by the jrl-cmakemodules.
 * Please see https://github.com/jrl-umi3218/jrl-cmakemodules/blob/master/config.hh.cmake for details.
*/

#ifndef PROXSUITE_CONFIG_HH
# define PROXSUITE_CONFIG_HH

// Package version (header).
# define PROXSUITE_VERSION_UNKNOWN_TAG 0 // Used to mention that the current version is unknown.
# define PROXSUITE_VERSION "0.3.2"
# define PROXSUITE_MAJOR_VERSION 0
# define PROXSUITE_MINOR_VERSION 3
# define PROXSUITE_PATCH_VERSION 2

#define PROXSUITE_VERSION_AT_LEAST(major, minor, patch) (PROXSUITE_MAJOR_VERSION>major || (PROXSUITE_MAJOR_VERSION>=major && \
                                                             (PROXSUITE_MINOR_VERSION>minor || (PROXSUITE_MINOR_VERSION>=minor && \
                                                                                                     PROXSUITE_PATCH_VERSION>=patch))))

#define PROXSUITE_VERSION_AT_MOST(major, minor, patch) (PROXSUITE_MAJOR_VERSION<major || (PROXSUITE_MAJOR_VERSION<=major && \
                                                            (PROXSUITE_MINOR_VERSION<minor || (PROXSUITE_MINOR_VERSION<=minor && \
                                                                                                     PROXSUITE_PATCH_VERSION<=patch))))

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define PROXSUITE_DLLIMPORT __declspec(dllimport)
#  define PROXSUITE_DLLEXPORT __declspec(dllexport)
#  define PROXSUITE_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define PROXSUITE_DLLIMPORT __attribute__ ((visibility("default")))
#   define PROXSUITE_DLLEXPORT __attribute__ ((visibility("default")))
#   define PROXSUITE_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define PROXSUITE_DLLIMPORT
#   define PROXSUITE_DLLEXPORT
#   define PROXSUITE_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef PROXSUITE_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PROXSUITE_DLLAPI
#  define PROXSUITE_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef proxsuite_EXPORTS
#   define PROXSUITE_DLLAPI PROXSUITE_DLLEXPORT
#  else
#   define PROXSUITE_DLLAPI PROXSUITE_DLLIMPORT
#  endif // PROXSUITE_EXPORTS
#  define PROXSUITE_LOCAL PROXSUITE_DLLLOCAL
# endif // PROXSUITE_STATIC
#endif //! PROXSUITE_CONFIG_HH
