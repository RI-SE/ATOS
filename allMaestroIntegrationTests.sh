#!/bin/bash
MAESTRODIR=$(pwd)
cd ${MAESTRODIR}/server/integration-tests
FAILURES=0
NUM_TESTS=0

echo "Running integration tests"
for f in *.sh; do
	if [ $(sh "$f" -H > /dev/null 2>&1) ]; then
		echo "Failed test ${f}"
		FAILURES=$((FAILURES+1))
	fi
	NUM_TESTS=$((NUM_TESTS+1))
done

SUCCESSES=$((NUM_TESTS-$FAILURES))
echo "Tests passed:  ${SUCCESSES} / ${NUM_TESTS}"

exit $FAILURES
