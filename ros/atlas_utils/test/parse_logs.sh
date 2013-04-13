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
set ylabel "Average publication rate (hz)"
set xlabel "Sample number"
plot '$avg' using 0:1 with lines
EOF
done

# Parse and plot the gzstats log
gnuplot <<EOF
set terminal png
set output '$HOME/.ros/plots/gzstats.png'
set ylabel "Real-time factor (sim/wall)"
set xlabel "Elapsed wall time (s)"
plot '$HOME/.ros/gzstats.log' using 3:1 with lines
EOF

# Parse and plot the controller_statistics log
rostopic echo -p -b $HOME/.ros/controller_statistics.bag /atlas/controller_statistics > $HOME/.ros/gen/controller_statistics.dat
$DIR/meanvar.py -c 4 -s , -g $HOME/.ros/gen/controller_statistics.dat > $HOME/.ros/gen/controller_statistics.dat.summary
gnuplot <<EOF
set terminal png
set output '$HOME/.ros/plots/controller_statistics.png'
set logscale xy
unset log x
set autoscale y
set autoscale y2
set y2tics
set ylabel "Percent of samples"
set y2label "Number of samples"
set xlabel "Command latency (sim ms)"
plot '$HOME/.ros/gen/controller_statistics.dat.summary' using 1:(100*\$3) with steps axes x1y1 title "Percent of samples", '$HOME/.ros/gen/controller_statistics.dat.summary' using 1:2 with steps axes x1y2 title "Number of samples"
EOF

if [ -e $run_name ]; then
  echo "Error: $run_name already exists"
  exit 1
fi
if [ -e $run_name.zip ]; then
  echo "Error: $run_name.zip already exists"
  exit 1
fi
ln -sf $HOME/.ros $run_name 
zip -q -r $run_name.zip $run_name
rm -rf $run_name
echo "Logs and plots are in $run_name.zip"

echo "Stats on real time factor:"
$DIR/meanvar.py ~/.ros/gzstats.log -s ,
