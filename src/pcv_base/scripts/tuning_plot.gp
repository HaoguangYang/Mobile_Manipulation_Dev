#!usr/bin/gnuplot

print "trace file name        :", ARG1

reset

set datafile separator ","

set xlabel "time [s]"
set ylabel "SI units"

set grid
set autoscale fix
set key outside right center

set style data linespoints

plot ARG1 using 1:(column(ARG2)) with lines,\
ARG1 using 1:(column(ARG3)) with lines,\
ARG1 using 1:(column(ARG4)) with lines,\
ARG1 using 1:(column(ARG5)) with lines,\
ARG1 using 1:(column(ARG6)) with lines,\
ARG1 using 1:(column(ARG7)) with lines,\

pause -1 "Hit return to close image and continue"

