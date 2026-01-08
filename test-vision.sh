#!/bin/bash
echo "Checking vision system status..."

ping wpilibpi.local && echo "\n\n\nConnected to WPILibPi!"
ping wpilibpi.local:1181 && echo "\n\n\nCamera stream is up!"
# -n ### = ping host for certain amount of time (sends 500 requests)
# -w ### = amount of milliseconds between pings (waits 100 ms)
