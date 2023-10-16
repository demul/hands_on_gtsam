# Detect Eigen version:
set(EIGEN_VER_H "${GTSAM_EIGEN_INCLUDE_FOR_BUILD}/Eigen/src/Core/util/Macros.h")

if(EXISTS ${EIGEN_VER_H})
    file(READ "${EIGEN_VER_H}" STR_EIGEN_VERSION)

    # Extract the Eigen version from the Macros.h file, lines "#define EIGEN_WORLD_VERSION  XX", etc...
    string(REGEX MATCH "EIGEN_WORLD_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_WORLD "${STR_EIGEN_VERSION}")
    string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_WORLD "${GTSAM_EIGEN_VERSION_WORLD}")

    string(REGEX MATCH "EIGEN_MAJOR_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_MAJOR "${STR_EIGEN_VERSION}")
    string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_MAJOR "${GTSAM_EIGEN_VERSION_MAJOR}")

    string(REGEX MATCH "EIGEN_MINOR_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_MINOR "${STR_EIGEN_VERSION}")
    string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_MINOR "${GTSAM_EIGEN_VERSION_MINOR}")

    set(GTSAM_EIGEN_VERSION "${GTSAM_EIGEN_VERSION_WORLD}.${GTSAM_EIGEN_VERSION_MAJOR}.${GTSAM_EIGEN_VERSION_MINOR}")

    message(STATUS "Found Eigen version: ${GTSAM_EIGEN_VERSION}")
else()
    message(WARNING "Cannot determine Eigen version, missing file: `${EIGEN_VER_H}`")
endif()

if(MSVC)
    if(BUILD_SHARED_LIBS)
        # mute eigen static assert to avoid errors in shared lib
        list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC EIGEN_NO_STATIC_ASSERT)
    endif()

    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE "/wd4244") # Disable loss of precision which is thrown all over our Eigen
endif()
