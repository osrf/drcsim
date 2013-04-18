#!/bin/bash

# Get the current working directory
cwd=`pwd`

stamp=`eval date +%d_%m_%Y_%R:%S`
tmp_dir="/tmp/.vrc_qual_tmp_dir-$stamp"
portal="http://vrcportal.osrfoundation.org"

purpose1="This script creates a qual_{n}.zip file(s) for submission to the VRC"
purpose2="qualifications.\n Two zip files are mandatory, qual_1.zip and "
purpose3="qual_2.zip. Two files are optional, qual_3.zip and qual_4.zip."
purpose4="It is your responsibilty to run this script with valid arguments, "
purpose5="and then upload the resulting zip files to the VRC Portal  "
purpose6="for submission. This ends my spiel."
purpose7="Good luck!\n"

# A simple help message
usage="mkqual.bash <qual task number> <Gazebo state log file> <score log file>"

# Make sure the correct number of parameters were supplied
if [ "$#" -ne "3" ]; then
  echo
  echo $purpose1
  echo $purpose2
  echo $purpose3
  echo $purpose4
  echo $purpose5
  echo $purpose6
  echo
  echo $purpose7
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

rm -rf $tmp_dir
mkdir $tmp_dir

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

echo "Important: You must upload qual_$1.zip to $portal."
sleep 2
