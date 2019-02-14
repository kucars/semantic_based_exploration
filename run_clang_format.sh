#!/bin/bash

find ./semantic_exploration -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format -i -style=file $1
