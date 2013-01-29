
###############################################################################
# Macro to turn a list into a string (why doesn't CMake have this
# built-in?)
macro(_list_to_string _string _list)
    set(${_string} "")
    foreach(_item ${_list})
        string(LENGTH "${${_string}}" _len)
        if(${_len} GREATER 0)
          set(${_string} "${${_string}} ${_item}")
        else(${_len} GREATER 0)
          set(${_string} "${_item}")
        endif(${_len} GREATER 0)
    endforeach(_item)
endmacro()


macro(invoke_rospack pkgname _prefix _varname)
  # Check that our cached location of rospack is valid.  It can be invalid
  # if rospack has moved since last time we ran, #1154.  If it's invalid,
  # search again.
  if(NOT EXISTS ${ROSPACK_EXE})
    message("Cached location of rospack is invalid; searching for rospack...")
    set(ROSPACK_EXE ROSPACK_EXE-NOTFOUND)
    # Only look in PATH for rospack, #3831
    find_program(ROSPACK_EXE NAMES rospack DOC "rospack executable" NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH NO_CMAKE_SYSTEM_PATH)
    if (NOT ROSPACK_EXE)
      message(FATAL_ERROR "Couldn't find rospack. Please source the appropriate ROS configuration file (e.g., /opt/ros/groovy/setup.sh)")
    endif(NOT ROSPACK_EXE)
  endif(NOT EXISTS ${ROSPACK_EXE})
  set(_rospack_invoke_result)
  execute_process(
    COMMAND ${ROSPACK_EXE} ${ARGN} ${pkgname}
    OUTPUT_VARIABLE _rospack_invoke_result
    ERROR_VARIABLE _rospack_err_ignore
    RESULT_VARIABLE _rospack_failed
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if (_rospack_failed)
    #set(_rospack_${_varname} "")
    #set(${_prefix}_${_varname} "" CACHE INTERNAL "")
    message("Failed to invoke ${ROSPACK_EXE} ${ARGN} ${pkgname}")
    message("${_rospack_err_ignore}")
    message("${_rospack_invoke_result}")
    message(FATAL_ERROR "\nFailed to invoke rospack to get compile flags for package '${pkgname}'.  Look above for errors from rospack itself.  Aborting.  Please fix the broken dependency!\n")
  else(_rospack_failed)
    separate_arguments(_rospack_invoke_result)
    set(_rospack_${_varname} ${_rospack_invoke_result})
    # We don't cache results that contain newlines, because
    # they make CMake's cache unhappy. This check should only affect calls
    # to `rospack plugins`, which we don't need to cache.
    if(_rospack_invoke_result MATCHES ".*\n.*")
      set(${_prefix}_${_varname} "${_rospack_invoke_result}")
    else(_rospack_invoke_result MATCHES ".*\n.*")
      set(${_prefix}_${_varname} "${_rospack_invoke_result}" CACHE INTERNAL "")
    endif(_rospack_invoke_result MATCHES ".*\n.*")
  endif(_rospack_failed)
endmacro()

macro(get_rospack_flags pkgname)
  # Get the include dirs
  set(_prefix ${pkgname})
  invoke_rospack(${pkgname} ${_prefix} "INCLUDE_DIRS" cflags-only-I)

  # Get the other cflags
  invoke_rospack(${pkgname} ${_prefix} temp cflags-only-other)
  _list_to_string(${_prefix}_CFLAGS_OTHER "${${_prefix}_temp}")
  set(${_prefix}_CFLAGS_OTHER ${${_prefix}_CFLAGS_OTHER} CACHE INTERNAL "")

  # Get the lib dirs
  invoke_rospack(${pkgname} ${_prefix} "LIBRARY_DIRS" libs-only-L)
  set(${_prefix}_LIBRARY_DIRS ${${_prefix}_LIBRARY_DIRS} CACHE INTERNAL "")

  # Get the libs
  invoke_rospack(${pkgname} ${_prefix} "LIBRARIES" libs-only-l)
  #
  # The following code removes duplicate libraries from the link line,
  # saving only the last one.
  #
  list(REVERSE ${_prefix}_LIBRARIES)
  list(REMOVE_DUPLICATES ${_prefix}_LIBRARIES)
  list(REVERSE ${_prefix}_LIBRARIES)

  # Get the other lflags
  invoke_rospack(${pkgname} ${_prefix} temp libs-only-other)
  _list_to_string(${_prefix}_LDFLAGS_OTHER "${${_prefix}_temp}")
  set(${_prefix}_LDFLAGS_OTHER ${${_prefix}_LDFLAGS_OTHER} CACHE INTERNAL "")
endmacro()

# gensrv processes srv/*.srv files into language-specific source files
macro(rosbuild_gensrv)
  # roslang_PACKAGE_PATH was set in rosbuild_init(), if roslang was found
  if(NOT roslang_PACKAGE_PATH)
    _rosbuild_warn("rosbuild_gensrv() was called, but the roslang package cannot be found. Service generation will NOT occur")
  endif(NOT roslang_PACKAGE_PATH)
  # Check whether there are any .srv files
  rosbuild_get_srvs(_srvlist)
  if(NOT _srvlist)
    _rosbuild_warn("rosbuild_gensrv() was called, but no .srv files were found")
  else(NOT _srvlist)
    file(WRITE ${PROJECT_SOURCE_DIR}/srv_gen/generated "yes")
    # Now set the mtime to something consistent.  We only want whether or not this file exists to matter
    # But we set it to the current time, because setting it to zero causes
    # annoying warning, #3396.
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "import os; os.utime('${PROJECT_SOURCE_DIR}/srv_gen/generated', (1, 1))"
      ERROR_VARIABLE _set_mtime_error
      RESULT_VARIABLE _set_mtime_failed
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_set_mtime_failed)
      message("[rosbuild] Error from calling to Python to set the mtime on ${PROJECT_SOURCE_DIR}/srv_gen/generated:")
      message("${_mtime_error}")
      message(FATAL_ERROR "[rosbuild] Failed to set mtime; aborting")
    endif(_set_mtime_failed)
  endif(NOT _srvlist)
  # Create target to trigger service generation in the case where no libs
  # or executables are made.
  add_custom_target(rospack_gensrv_all ALL)
  add_dependencies(rospack_gensrv_all rospack_gensrv)
  # Make the precompile target, on which libraries and executables depend,
  # depend on the message generation.
  add_dependencies(rosbuild_precompile rospack_gensrv)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/srv_gen/cpp/include)
endmacro(rosbuild_gensrv)

# genmsg processes msg/*.msg files into language-specific source files
macro(rosbuild_genmsg)
  # roslang_PACKAGE_PATH was set in rosbuild_init(), if roslang was found
  if(NOT roslang_PACKAGE_PATH)
    _rosbuild_warn("rosbuild_genmsg() was called, but the roslang package cannot be found.  Message generation will NOT occur")
  endif(NOT roslang_PACKAGE_PATH)
  # Check whether there are any .srv files
  rosbuild_get_msgs(_msglist)
  if(NOT _msglist)
    _rosbuild_warn("rosbuild_genmsg() was called, but no .msg files were found")
  else(NOT _msglist)
    file(WRITE ${PROJECT_SOURCE_DIR}/msg_gen/generated "yes")
    # Now set the mtime to something consistent.  We only want whether or not this file exists to matter
    # But we set it to the current time, because setting it to zero causes
    # annoying warning, #3396.
    execute_process(
      COMMAND ${PYTHON_EXECUTABLE} -c "import os; os.utime('${PROJECT_SOURCE_DIR}/msg_gen/generated', (1, 1))"
      ERROR_VARIABLE _set_mtime_error
      RESULT_VARIABLE _set_mtime_failed
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(_set_mtime_failed)
      message("[rosbuild] Error from calling to Python to set the mtime on ${PROJECT_SOURCE_DIR}/msg_gen/generated:")
      message("${_mtime_error}")
      message(FATAL_ERROR "[rosbuild] Failed to set mtime; aborting")
    endif(_set_mtime_failed)
  endif(NOT _msglist)
  # Create target to trigger message generation in the case where no libs
  # or executables are made.
  add_custom_target(rospack_genmsg_all ALL)
  add_dependencies(rospack_genmsg_all rospack_genmsg)
  # Make the precompile target, on which libraries and executables depend,
  # depend on the message generation.
  add_dependencies(rosbuild_precompile rospack_genmsg)
  # add in the directory that will contain the auto-generated .h files
  include_directories(${PROJECT_SOURCE_DIR}/msg_gen/cpp/include)
endmacro(rosbuild_genmsg)


macro(rosbuild_get_srvs srvvar)
  file(GLOB _srv_files RELATIVE "${PROJECT_SOURCE_DIR}/srv" "${PROJECT_SOURCE_DIR}/srv/*.srv")
  set(${srvvar} ${_ROSBUILD_GENERATED_SRV_FILES})
  # Loop over each .srv file, establishing a rule to compile it
  foreach(_srv ${_srv_files})
    # Make sure we didn't get a bogus match (e.g., .#Foo.srv, which Emacs
    # might create as a temporary file).  the file()
    # command doesn't take a regular expression, unfortunately.
    if(${_srv} MATCHES "^[^\\.].*\\.srv$")
      list(APPEND ${srvvar} ${_srv})
    endif(${_srv} MATCHES "^[^\\.].*\\.srv$")
  endforeach(_srv)
endmacro(rosbuild_get_srvs)




macro(_rosbuild_warn)
  message("[rosbuild] WARNING: " ${ARGV})
endmacro(_rosbuild_warn)


