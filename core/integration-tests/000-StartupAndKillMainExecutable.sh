#!/bin/sh
EXECDIR=../../build/bin
EXECNAME=Core
SLEEP_TIME_START=3
SLEEP_TIME_KILL=1
RESULT=0
EXPECTED_PIDS[0]="Central"
EXPECTED_PIDS[1]="SystemControl"
EXPECTED_PIDS[2]="ObjectControl"
EXPECTED_PIDS[3]="TimeControl"
EXPECTED_PIDS[4]="JournalControl"

# Start the main executable
rm -rf /dev/mqueue/*
$EXECDIR/$EXECNAME &
sleep $SLEEP_TIME_START
RUNNING_PIDS="$(pgrep ${EXECNAME} | wc -l)"
if [ $RUNNING_PIDS -lt ${#EXPECTED_PIDS[@]} ]; then
	echo "All processes not running after ${SLEEP_TIME_START} s"
	RESULT=1
fi

killall -s SIGINT $EXECNAME
sleep $SLEEP_TIME_KILL
RUNNING_PIDS="$(pgrep ${EXECNAME} | wc -l)"
if [ $RUNNING_PIDS -ne 0 ]; then
	echo "Failed to kill all processes via SIGINT. Still running:"
	pgrep $EXECNAME
	killall $EXECNAME
	RESULT=1
fi

exit $RESULT
