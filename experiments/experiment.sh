#!/bin/bash

PATH_KOORD_JAR="koord-0.1-jar-with-dependencies.jar"
PATH_APP_KRD=$1
PATH_CONFIG=$2
PATH_APP_PY=$(mktemp)

TIMEOUT="120s"


if [ ! -f "$PATH_KOORD_JAR" ]; then
    echo "Cannot find JAR file $PATH_KOORD_JAR for compiling Koord programs"
    URL_KOORD="https://github.com/cyphyhouse/KoordLanguage/releases"
    echo "You can find available Koord releases at ${URL_KOORD}"
    exit
fi

if [ ! -f "$PATH_APP_KRD" ]; then
    echo "Please specify a Koord program"
    exit
fi

if [ ! -f "$PATH_CONFIG" ]; then
    echo "Please specify a configuration file"
    exit
fi

# TODO Start up Cymulator

# Compile the given Koord program to Python
java -jar $PATH_KOORD_JAR $PATH_APP_KRD $PATH_APP_PY

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

