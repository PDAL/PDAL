#!/bin/bash

# all argument are considered as a program to call (with its arguments),
# the last argument is read from stdin via '<'

set -e

arr=( "$@" )

input=${arr[${#arr[@]}-1]}
unset 'arr[${#arr[@]}-1]'

${arr[@]} < $input
