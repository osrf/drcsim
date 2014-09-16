find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${PROJECT_SOURCE_DIR}/doc/drcsim.in
                 ${PROJECT_BINARY_DIR}/drcsim.dox @ONLY)

  # FIXME TODO move this to drcsim_gazebo_plugins
  # Get the AtlasSimInterface PATH non depending upon versions
  file (GLOB atlas_dir "${PROJECT_SOURCE_DIR}/AtlasSimInterface*/doc/html")

  add_custom_target(doc
    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/drcsim.dox
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    # Copy already generated AtlasInterface doxygen
    COMMAND cp -R ${atlas_dir}
            ${PROJECT_BINARY_DIR}/doxygen/html/atlas_interface
    COMMAND cp ${PROJECT_SOURCE_DIR}/doc/drcsim_logo.png
            ${PROJECT_BINARY_DIR}/doxygen/html
    COMMAND cp ${PROJECT_SOURCE_DIR}/doc/search.js
            ${PROJECT_BINARY_DIR}/doxygen/html/search
    COMMAND make -C ${PROJECT_BINARY_DIR}/doxygen/latex
    COMMAND mv ${PROJECT_BINARY_DIR}/doxygen/latex/refman.pdf
            ${PROJECT_BINARY_DIR}/doxygen/latex/drcsim-${drcsim_model_resources_VERSION}.pdf

    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

