#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

USAGE="parse_hz_logs.sh <run_name>"

if [ $# -ne 1 ]; then
    echo $USAGE
    exit 1
fi

run_name=$1

# Parse and plot the hz logs
mkdir -p $HOME/.ros/gen
mkdir -p $HOME/.ros/plots
for f in `find $HOME/.ros -name "hz_*stdout.log"`; do
  b=`basename $f`
  avg=$HOME/.ros/gen/$b.avg
  png=$HOME/.ros/plots/$b.png
  grep average $f | awk {'print $3'} > $avg
  gnuplot <<EOF
set terminal png
set output '$png'
plot '$avg' using 0:1 with lines
EOF
done

# Parse and plot the gzstats log
gnuplot <<EOF
set terminal png
set output '$HOME/.ros/plots/gzstats.png'
plot '$HOME/.ros/gzstats.log' using 3:1 with lines
EOF

# Parse and plot the controller_statistics log
#rostopic echo -p -b $HOME/.ros/controller_statistics.bag /atlas/controller_statistics > $HOME/.ros/gen/controller_statistics.dat
#$DIR/meanvar.py -c 4 -s , -g $HOME/.ros/gen/controller_statistics.dat > $HOME/.ros/gen/controller_statistics.dat.summary
#gnuplot <<EOF
#set terminal png
#set output '$HOME/.ros/plots/gzstats.png'
#plot '$HOME/.ros/gzstats.log' using 3:1 with lines
#EOF

if [ -f $run_name.zip ]; then
  echo "Error: $run_name.zip already exists"
  exit 1
fi
zip -q -r $run_name.zip $HOME/.ros
echo "Logs and plots are in $run_name.zip"
