#!/bin/bash
# Settings for GNU indent
CODE_STYLE_OPTIONS="--k-and-r-style"

# Disable generating backup files
export VERSION_CONTROL="none"

# Find all source files and pass them to GNU indent
find . -iname "*.c" -a -not -ipath "*ASN1*" -a -not -ipath "*CMakeFiles*" -a -not -ipath "*util/*" | xargs indent ${CODE_STYLE_OPTIONS}
