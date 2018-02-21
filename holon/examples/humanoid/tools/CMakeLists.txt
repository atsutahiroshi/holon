file(GLOB HOLON_EXAMPLES_HUMANOID_TOOLS_FILES *.py)

foreach(path ${HOLON_EXAMPLES_HUMANOID_TOOLS_FILES})
  get_filename_component(file ${path} NAME)
  configure_file(${file} ${file} COPYONLY)
endforeach()
