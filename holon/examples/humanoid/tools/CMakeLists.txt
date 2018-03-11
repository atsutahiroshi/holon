file(GLOB scripts *.py)

foreach(path ${scripts})
  get_filename_component(file ${path} NAME)
  configure_file(${file} ${file} COPYONLY)
endforeach()
