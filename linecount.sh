#!/bin/sh
find . -name "*.cpp" -o -name "*.h" -o -name "*.hpp" | xargs wc -l
