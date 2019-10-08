#!/bin/bash

if ! git diff-index --quiet HEAD --; then
	echo "Then!"
else
	echo "Else"
fi
