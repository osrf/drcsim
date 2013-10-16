#!/bin/sh

SCRIPT_PATH=$(readlink -f $0 | sed -e 's:/tools/code_check.sh$::g' )

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
  # Assuming that Jenkins called, the `build` directory is a sibling to the src dir
  builddir=../build
else
  # This is a heuristic guess; not every developer puts the `build` dir in the src dir
  builddir=./build
fi

#cppcheck
if [ $xmlout -eq 1 ]; then
  (cppcheck --enable=all -q --xml `find $SCRIPT_PATH/drcsim_gazebo_plugins $SCRIPT_PATH/drcsim_gazebo_ros_plugins -name "*.cc"` --check-config) 2> $xmldir/cppcheck.xml
else
  cppcheck --enable=all -q `find $SCRIPT_PATH/drcsim_gazebo_plugins $SCRIPT_PATH/drcsim_gazebo_ros_plugins -name "*.cc"` --check-config
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (find $SCRIPT_PATH/drcsim_gazebo_plugins $SCRIPT_PATH/drcsim_gazebo_ros_plugins -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1) | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  find $SCRIPT_PATH/drcsim_gazebo_plugins $SCRIPT_PATH/drcsim_gazebo_ros_plugins -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1
fi
