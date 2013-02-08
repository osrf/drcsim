#!/bin/sh

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
  (cppcheck --enable=all -q --xml `find ./plugins ./ros -name "*.cc"` --check-config) 2> $xmldir/cppcheck.xml
else
  cppcheck --enable=all -q `find ./plugins ./ros -name "*.cc"` --check-config
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (find ./plugins ./ros -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1) | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  find ./plugins ./ros -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1
fi
