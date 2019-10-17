#!/bin/bash
MAESTRODIR=$(pwd)
cd ${MAESTRODIR}/server/integration-tests
FAILURES=0
NUM_TESTS=0

echo "Running integration tests"
for f in $(pwd)/*; do
	if [ ${f: -3} == ".py" ]; then
		echo "Running ${f}"
		python3 "$f"
		if [ "$?" != "0" ]; then
			echo "Failed test ${f}"
			FAILURES=$((FAILURES+1))
		fi
		NUM_TESTS=$((NUM_TESTS+1))
	elif [ ${f: -3} == ".sh" ]; then
		echo "Running ${f}"
		if [ $(sh "$f" -H > /dev/null 2>&1) ]; then
			echo "Failed test ${f}"
			FAILURES=$((FAILURES+1))
		fi
		NUM_TESTS=$((NUM_TESTS+1))
	fi
done

SUCCESSES=$((NUM_TESTS-$FAILURES))
echo "Tests passed:  ${SUCCESSES} / ${NUM_TESTS}"

exit $FAILURES
