#!/bin/bash
MAESTRODIR=$(pwd)
cd ${MAESTRODIR}/core/extensive-tests
FAILURES=0
NUM_TESTS=0
NUMBER_REGEX='^[0-9]+'

echo "Running integration tests"
for f in $(pwd)/*; do
	rm -f /dev/mqueue/*
	fname="$(basename -- ${f})"
	if [[ ${fname:0:3} =~ $NUMBER_REGEX ]] ; then
		echo "Running ${fname}"
		if [ ${fname: -3} == ".py" ]; then
			python3 "$f"
			if [ "$?" != "0" ]; then
				echo "Failed test ${fname}"
				FAILURES=$((FAILURES+1))
			fi
			NUM_TESTS=$((NUM_TESTS+1))
		elif [ ${fname: -3} == ".sh" ]; then
			if [ $(sh "$f" -H > /dev/null 2>&1) ]; then
				echo "Failed test ${fname}"
				FAILURES=$((FAILURES+1))
			fi
			NUM_TESTS=$((NUM_TESTS+1))
		else
			echo "Skipping ${fname}"
		fi
	else
		echo "Skipping ${fname}"
	fi
done

SUCCESSES=$((NUM_TESTS-$FAILURES))
echo "Tests passed:  ${SUCCESSES} / ${NUM_TESTS}"

exit $FAILURES
