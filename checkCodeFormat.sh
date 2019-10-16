#!/bin/bash

if ! git diff-index --quiet HEAD --; then
	echo "Uncommitted changes detected: please commit changes before checking code format"
	exit 1
fi

./formatSourceFiles.sh
git diff > formatDiff.patch

# Reset changes made
find . -iname "*.c" -a -not -ipath "*ASN1*" -a -not -ipath "*CMakeFiles*" -a -not -ipath "*util/*" | xargs git checkout --

if [ ! -s formatDiff.patch ]; then
	# It was empty
	rm formatDiff.patch
	echo "No code formatting errors detected"
	exit 0
else
	echo "Formatting errors detected. Attempting to fix ..."
	rm formatDiff.patch

	./formatSourceFiles.sh
	./formatSourceFiles.sh

	git diff > formatDiff.patch

	find . -iname "*.c" -a -not -ipath "*ASN1*" -a -not -ipath "*CMakeFiles*" -a -not -ipath "*util/*" | xargs git checkout --

	if [ ! -s formatDiff.patch ]; then
		rm formatDiff.patch
		echo "No code formatting errors detected after rerun"
		exit 0
	else
		# Very wordy printout:
		cat formatDiff.patch
		rm formatDiff.patch
		exit 1
	fi
fi
