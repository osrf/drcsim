find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/drcsim.in
                 ${CMAKE_BINARY_DIR}/drcsim.dox @ONLY)

  # Get the AtlasSimInterface PATH non depending upon versions
  file (GLOB atlas_dir "${CMAKE_SOURCE_DIR}/plugins/AtlasSimInterface*/doc/html")

  add_custom_target(doc

    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/drcsim.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    # Copy already generated AtlasInterface doxygen
    COMMAND cp -R ${atlas_dir}
            ${CMAKE_BINARY_DIR}/doxygen/html/atlas_interface
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/drcsim_logo.png
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/search.js
            ${CMAKE_BINARY_DIR}/doxygen/html/search
    COMMAND make -C ${CMAKE_BINARY_DIR}/doxygen/latex
    COMMAND mv ${CMAKE_BINARY_DIR}/doxygen/latex/refman.pdf
            ${CMAKE_BINARY_DIR}/doxygen/latex/drcsim-${DRC_SIM_VERSION_FULL}.pdf

    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

