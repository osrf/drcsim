#!/bin/bash
urdfFile=`rospack find $1`/$1.urdf
echo link,mass,ixx,iyy,izz,ixy,ixz,iyz
for suffix in \
 `xmlstarlet sel -t -v '//link/@name' $urdfFile | \
  grep '^r_' | sed -e 's@^r@@'`
do
  xmlstarlet sel -t -o "r$suffix," \
          -t -v "//link[@name='r$suffix']/inertial/mass/@value"  -t -o ',' \
          -t -v "//link[@name='r$suffix']/inertial/inertia/@ixx" -t -o ',' \
          -t -v "//link[@name='r$suffix']/inertial/inertia/@iyy" -t -o ',' \
          -t -v "//link[@name='r$suffix']/inertial/inertia/@izz" -t -o ',' \
          -t -v "//link[@name='r$suffix']/inertial/inertia/@ixy" -t -o ',' \
          -t -v "//link[@name='r$suffix']/inertial/inertia/@ixz" -t -o ',' \
          -t -v "//link[@name='r$suffix']/inertial/inertia/@iyz" $urdfFile

  xmlstarlet sel -t -o "l$suffix," \
          -t -v "//link[@name='l$suffix']/inertial/mass/@value"  -t -o ',' \
          -t -v "//link[@name='l$suffix']/inertial/inertia/@ixx" -t -o ',' \
          -t -v "//link[@name='l$suffix']/inertial/inertia/@iyy" -t -o ',' \
          -t -v "//link[@name='l$suffix']/inertial/inertia/@izz" -t -o ',' \
          -t -v "//link[@name='l$suffix']/inertial/inertia/@ixy" -t -o ',' \
          -t -v "//link[@name='l$suffix']/inertial/inertia/@ixz" -t -o ',' \
          -t -v "//link[@name='l$suffix']/inertial/inertia/@iyz" $urdfFile
done
