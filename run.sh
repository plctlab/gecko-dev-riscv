#!/bin/sh

# Executing command: sh run.sh
# User Input for Debug / Optimized Build
echo -n "Enter build type: \n 1-Debug \n 2-Optimized \n=>"
read VAR

# exporting alias for Mozilla Build Configuration Files
export alias get_moz_debug="export ./mozconfigs/debug/MOZCONFIG"
export alias get_moz_optimized="export ./mozconfigs/debug/MOZCONFIG"

# calling alias for suitable Build Configuration
if [ $VAR -eq 1 ]
then
    echo "Getting Ready for Debug Build ..."
    get_moz_debug
else
    echo "Getting Ready for Optimized Build ..."
    get_moz_optimized
fi

# Installing dependenices and getting environment ready for build
# Choose Spidermonkey from the Environment creation list
./mach bootstrap

# Build Spidermonkey as per set configuration
./mach build

# Running jsshell
./mach run