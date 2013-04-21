#!/bin/bash

# Get the current working directory
cwd=`pwd`

portal="http://vrcportal.osrfoundation.org"

purpose="This script creates a qual_{n}.zip file(s) for submission to the VRC\n\
qualifications. Two zip files are mandatory, qual_1.zip and\n\
qual_2.zip. Two files are optional, qual_3.zip and qual_4.zip.\n\
It is your responsibilty to run this script with valid arguments,\n\
and then upload the resulting zip files to the VRC Portal\n\
for submission. This ends my spiel.\n\
Good luck!\n\n"

# A simple help message
usage="mkqual.bash <qual task number> <Gazebo state log file> <score log file>"

# Make sure the correct number of parameters were supplied
if [ "$#" -ne "3" ]; then
  echo
  echo -e $purpose
  echo
  echo "VRC Portal: $portal"
  echo
  echo "Usage:"
  echo $usage 
  exit 1
fi

# The first option must be a value between 1 and 4 inclusive.
if [ $1 -lt "1" ] || [ $1 -gt "4" ]; then
  echo $usage
  echo "First option must be the number of Qual task: 1, 2, 3, or 4."
  exit 1
fi

# The second option must be a valid file
if [ ! -e $2 ]; then
  echo $usage
  echo "Gazebo state log file[$2] does not exist"
  exit 1
fi

# The third option must be a valid file
if [ ! -e $3 ]; then
  echo $usage
  echo "Score log file[$3] does not exist"
  exit 1
fi

if [ -e $cwd/vrc_qual_$1.zip ]; then
  echo "The file $cwd/vrc_qual_$1.zip already exists and I won't overwrite it.  Aborting."
  exit 1
fi

tmp_dir=`mktemp -d`


echo "Filtering the Gazebo state log file. This may take many minutes."
echo "If an error message appears, then you should recreate the log file."

echo -n "Filtering..."

# This is a hack to fix an occasional error by Gazebo.
if ! gzlog info $2 &> /dev/null; then
  echo "</gazebo_log>" >> $2
fi

# Filter the state log file into the work directory
gzlog echo $2 -z 30 --filter *.pose/*.pose > $tmp_dir/state.log
echo "done."

# Copy the score file to the work directory
cp $3 $tmp_dir

# Create the final zip file
echo "Creating final zip file = vrc_qual_$1.zip"
cd $tmp_dir
ls | grep -v vrc_manifest.txt > $tmp_dir/vrc_manifest.txt
zip vrc_qual_$1.zip *
mv $tmp_dir/vrc_qual_$1.zip $cwd

rm -rf $tmp_dir

echo "Important: You must upload vrc_qual_$1.zip to $portal."
sleep 2
