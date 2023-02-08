project(envire_mars_ode_physics)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "This library ...")
cmake_minimum_required(VERSION 2.6)

find_package(Rock)

include(FindPkgConfig)

find_package(lib_manager)
lib_defaults()
define_module_info()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} "${PROJECT_SOURCE_DIR}/cmake")

set(DEFAULT_CONFIG_DIR "${CMAKE_INSTALL_PREFIX}/configuration/mars_default" CACHE STRING "The Default config dir to load")
add_definitions(-DDEFAULT_CONFIG_DIR=\"${DEFAULT_CONFIG_DIR}\")


MACRO(CMAKE_USE_FULL_RPATH install_rpath)
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    SET(CMAKE_INSTALL_RPATH ${install_rpath})
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
ENDMACRO(CMAKE_USE_FULL_RPATH)
CMAKE_USE_FULL_RPATH("${CMAKE_INSTALL_PREFIX}/lib")

#find_package(Doxygen)
#if(DOXYGEN_FOUND)
#configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
#add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMENT "Generating API documentation with Doxygen" VERBATIM)
#endif(DOXYGEN_FOUND)

#rock_find_cmake(Boost)

#Get linker and compiler flags from pkg-config
pkg_check_modules(PKGCONFIG REQUIRED
			    lib_manager
			    mars_interfaces
          mars_utils
          configmaps
          envire_core
          envire_smurf
          mars_ode_physics
)

include_directories(${PKGCONFIG_INCLUDE_DIRS})
link_directories(${PKGCONFIG_LIBRARY_DIRS})
add_definitions(${PKGCONFIG_CFLAGS_OTHER})  #flags excluding the ones with -I

#add_definitions(-DODE11=1 -DdDOUBLE)
#add_definitions(-DFORWARD_DECL_ONLY=1)

foreach(DIR ${CFG_MANAGER_INCLUDE_DIRS})
    set(ADD_INCLUDES "${ADD_INCLUDES} -I${DIR}")
endforeach(DIR)

include_directories( ${CMAKE_CURRENT_BINARY_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src )

set(SOURCES_H
       src/EnvireOdePhysicsPlugins.hpp
)

set(TARGET_SRC
       src/EnvireOdePhysicsPlugins.cpp
)

#cmake variables
#configure_file(${CMAKE_SOURCE_DIR}/config.h.in ${CMAKE_BINARY_DIR}/config.h @ONLY)
include_directories("${CMAKE_BINARY_DIR}")
add_library(${PROJECT_NAME} SHARED ${TARGET_SRC})

IF (WIN32)
  set(WIN_LIBS -lwsock32 -lwinmm -lpthread)
#  SET_TARGET_PROPERTIES(mars PROPERTIES LINK_FLAGS -Wl,--stack,0x1000000)
ENDIF (WIN32)

set(_INSTALL_DESTINATIONS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION lib
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME}
            ${PKGCONFIG_LIBRARIES}
            ${WIN_LIBS}
)



if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)

set(_INSTALL_DESTINATIONS
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION lib
)

# Install the library
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${SOURCES_H} DESTINATION include/${PROJECT_NAME})


configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
