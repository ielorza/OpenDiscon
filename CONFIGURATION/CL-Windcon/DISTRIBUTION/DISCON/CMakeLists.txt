set (OPENDISCON_SOURCES ${OPENDISCON_SOURCES} ${PROJECT_SOURCE_DIR}/CONFIGURATION/${CONFIGURATION}/src/discon/discon.c)

# shared OpenDiscon library
include_directories ("${OPENDISCON_INCLUDE_DIRS}")
include_directories ("${OPENWITCON_INCLUDE_DIRS}")
include_directories ("${PROJECT_BINARY_DIR}")
include (GenerateExportHeader)
add_library (OpenDiscon SHARED ${OPENDISCON_SOURCES})
GENERATE_EXPORT_HEADER (OpenDiscon
	BASE_NAME OpenDiscon
	EXPORT_MACRO_NAME OpenDiscon_EXPORT
	EXPORT_FILE_NAME OpenDiscon_EXPORT.h
	STATIC_DEFINE OpenDiscon_BUILT_AS_STATIC
)
