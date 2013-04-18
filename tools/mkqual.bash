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
  exit
fi

# The first option must be a value between 1 and 4 inclusive.
if [ $1 -lt "1" ] || [ $1 -gt "4" ]; then
  echo $usage
  echo "First option must be the number of Qual task: 1, 2, 3, or 4."
  exit
fi

# The second option must be a valid file
if [ ! -e $2 ]; then
  echo $usage
  echo "Gazebo state log file[$2] does not exist"
  exit
fi

# The third option must be a valid file
if [ ! -e $3 ]; then
  echo $usage
  echo "Score log file[$3] does not exist"
  exit
fi

tmp_dir="mktemp"

echo "Filtering the Gazebo state log file. This may take many minutes."
echo "If an error message appears, then you should recreate the log file."
sleep 3

# Filter the state log file into the work directory
gzlog echo $2 -z 30 --filter *.pose/*.pose > $tmp_dir/state.log

# Copy the score file to the work directory
cp $3 $tmp_dir
cd $tmp_dir

# Create the final zip file
echo "Creating final zip file = qual_$1.zip"
zip qual_$1.zip *.log
mv $tmp_dir/*.zip $cwd

rm -rf $tmp_dir

echo "Important: You must upload qual_$1.zip to $portal."
sleep 2
