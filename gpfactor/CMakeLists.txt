set(gpfactor_dirs gp)
set(gpfactor_srcs)

# Loop over only gp folder
foreach(subdir ${gpfactor_dirs})
    file(GLOB subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h")  
    list(APPEND gpfactor_srcs ${subdir_srcs})
    message(STATUS "Building Module: ${subdir_srcs}")
    add_subdirectory(${subdir})
endforeach(subdir)

add_library(${PROJECT_NAME} SHARED ${gpfactor_srcs})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${GTSAM_LIBS})
install(TARGETS ${PROJECT_NAME} EXPORT gpfactor-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)


