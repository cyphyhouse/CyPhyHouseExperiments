#!/bin/bash

PATH_APP_PY=$1
PATH_CONFIG=$2

TIMEOUT="1000s"


if [ ! -f "$PATH_APP_PY" ]; then
    echo "Please specify a Python program"
    exit
fi

if [ ! -f "$PATH_CONFIG" ]; then
    echo "Please specify a configuration file"
    exit
fi

# TODO Start up Cymulator

# TODO Start up background processes for collecting statistics


function finish
{
    # TODO kill background processes
    echo "Terminating background processes"
}
trap finish EXIT

# Execute Koord application with TIMEOUT
PYTHONPATH="$PYTHONPATH:.."  # Set temporary PYTHONPATH to import cph
timeout --foreground -s INT $TIMEOUT python3 -m cph $PATH_APP_PY $PATH_CONFIG
