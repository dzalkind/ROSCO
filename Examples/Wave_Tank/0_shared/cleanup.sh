#!/bin/bash

# Check if directory argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <directory>"
    exit 1
fi

# Check if the provided argument is a valid directory
if [ ! -d "$1" ]; then
    echo "Error: '$1' is not a valid directory"
    exit 1
fi

# Remove all .out and .sum files from the specified directory
find "$1" -name "*.out*" -type f -delete
find "$1" -name "*.sum" -type f -delete
find "$1" -name "*.dbg*" -type f -delete
find "$1" -name "*.ech" -type f -delete
find "$1" -name "*.json" -type f -delete
find "$1" -name "*.sum.yaml" -type f -delete
find "$1" -name "case_matrix*" -type f -delete

echo "Removed all output files from $1"