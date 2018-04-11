cmake_minimum_required (VERSION 3.0.0)
project (OpenDiscon)

# choose a configuration
set( CONFIGURATION "CL-Windcon" CACHE STRING
"Choose a configuration.\n\
Available configurations are:\n\
CL-Windcon" )

# run configuration-specific cmake script
include( ${PROJECT_SOURCE_DIR}/CONFIGURATION/${CONFIGURATION}/CMakeLists.txt )
