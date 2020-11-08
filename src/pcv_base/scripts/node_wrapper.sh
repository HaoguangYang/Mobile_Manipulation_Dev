#!/bin/bash

pkgName=pcv_base
nodeName=pcv_base_node

trap on_interrupt INT TERM KILL

on_interrupt ()
{
    echo "INTERRUPT CAUGHT"    
    #kill -INT $1 # Kills the first thing you started
    sudo kill -INT $(ps aux | grep "$nodeName" | awk '{print $2}')	# Terminate any child processes
    wait # for this process to end
}

rosrun --prefix 'sudo -E' $pkgName $nodeName &

wait
