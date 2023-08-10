#!/bin/bash
# This script formats all code recursively according to the google style.

if [ "$#" -eq 0 ]; then
    echo "Usage: $0 <dir>"
    echo ""
    echo "ERROR: At least one source file or one directory must be provided!"

    exit 1
fi

# find clang format binary
for clang_binary in clang-format-10; do
	[ -x "$(command -v ${clang_binary})" ] && CLANG_FORMAT_BINARY=${clang_binary}
done
if [ -z $CLANG_FORMAT_BINARY ]; then
	echo "Error: clang-format binary not found"
	exit 1
fi

for arg in "$@"
do
    find $arg  \( -name extern -o -name build \) -prune -type f -o -iname '*.h' -o -iname '*.cpp' -o -iname '*.hpp' \
		| xargs ${CLANG_FORMAT_BINARY} -style=file -i
done

echo "Done formatting"
