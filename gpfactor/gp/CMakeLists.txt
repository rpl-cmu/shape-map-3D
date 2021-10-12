# Install headers
file(GLOB gp_headers "*.h")
install(FILES ${gp_headers} DESTINATION include/gpfactor/gp)
