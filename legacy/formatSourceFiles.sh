#!/bin/bash
#### Settings for GNU indent
TAB_SIZE=4
LINE_LENGTH=110
# Blank lines
CODE_STYLE_OPTIONS="--blank-lines-after-declarations --blank-lines-after-procedures"
# If-else statements
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --braces-on-if-line --dont-cuddle-else"
# Do-while statements
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --dont-cuddle-do-while"
# Switch-case statements
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --case-indentation0 --case-brace-indentation0"
# Typecasts
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --no-space-after-cast"
# Sizeof
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --blank-before-sizeof"
# Structs
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --braces-on-struct-decl-line"
# Functions/procedures
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --dont-break-procedure-type --braces-on-func-def-line"
# Parentheses indentation on newline
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --continue-at-parentheses"
# Indentation
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --indent-level${TAB_SIZE} --tab-size${TAB_SIZE}"
# Column cap
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --line-length${LINE_LENGTH}"
# Function call appearance
CODE_STYLE_OPTIONS="${CODE_STYLE_OPTIONS} --no-space-after-function-call-names"

# Disable generating backup files
export VERSION_CONTROL="none"

# Find all source files and pass them to GNU indent
find . -iname "*.c" -a -not -ipath "*ASN1*" -a -not -ipath "*CMakeFiles*" -a -not -ipath "*util/*" | xargs indent ${CODE_STYLE_OPTIONS}
