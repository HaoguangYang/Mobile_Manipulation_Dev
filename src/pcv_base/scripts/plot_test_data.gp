#!usr/bin/gnuplot

print "trace file name        :", ARG1
print "actual data column name to plot    :", ARG2
print "command data column name to plot    :", ARG3

reset

set datafile separator ","

set xlabel "time [s]"
set ylabel "SI units"

set grid
set autoscale fix
set key outside right center

set style data linespoints

plot ARG1 using 1:(column(ARG2)),\
ARG1 using 1:(column(ARG3))

pause -1 "Hit return to close image and continue"

