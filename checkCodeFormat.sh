#!/bin/bash

if ! git diff-index --quiet HEAD --; then
	echo "Uncommitted changes detected: please commit changes before checking code format"
	exit 1
fi

./formatSourceFiles.sh
git diff > formatDiff.patch
# Todo: remove formatting diffs

if [ ! -s formatDiff.patch ]; then
	# It was empty
	rm formatDiff.patch
	echo "No code formatting errors detected"
	exit 0
else
	echo "Formatting errors detected:"
	cat formatDiff.patch
	rm formatDiff.patch
	exit 1
fi
