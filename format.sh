#!/bin/bash

# sudo apt-get install clang-format version 10
find ./ -regex '.*\.cc\|.*\.cpp|.*\.h\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format-10 -i --style=file
