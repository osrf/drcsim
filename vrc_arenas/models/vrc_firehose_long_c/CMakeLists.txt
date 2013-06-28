set(MODEL_NAME vrc_firehose_long_c)
set (files 
  model.config
  model.sdf
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/models/${MODEL_NAME}/)
