function makeOpenDiscon

mex -output OpenDiscon -I${OPENWITCON_INCLUDE_DIRS} -I${OPENDISCON_INCLUDE_DIRS} ${OPENWITCON_SOURCES} ${OPENDISCON_SOURCES}
