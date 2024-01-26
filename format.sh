#!/bin/bash

# sudo apt-get install clang-format
find ./ -regex '.*\.cc\|.*\.cpp|.*\.h\|.*\.proto' -and -not -regex '.*\.pb\.cc\|.*\.pb\.h' | xargs clang-format -i --style=file
