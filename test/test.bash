#!/bin/bash
# SPDX-FileCopyrightText: 2025 Keiichiro Kobayashi
# SPDX-License-Identifier: BSD-3-Clause
dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
if [ $? -ne 0 ]; then
    echo "Build failed"
    exit 1
fi

source install/setup.bash

LOG_FILE=/tmp/weather_forecast.log

echo "Running weather_forecast and saving logs to $LOG_FILE"
timeout 20 bash -c "ros2 run robosysasgmt2 weather_forecast > $LOG_FILE 2>&1"

echo "Checking log file..."
if [ -s $LOG_FILE ]; then
    echo "Log file is not empty. Checking content..."
    cat $LOG_FILE | grep -i "Published: "
    if [ $? -eq 0 ]; then
        echo "Test passed"
        exit 0
    else
        echo "Test failed"
        exit 1
    fi
else
    echo "Log file is empty or not created."
    exit 1
fi

