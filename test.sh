#!/bin/bash
set -e

# Untrack and remove all LFS files from the repository
git lfs ls-files | while read -r line; do
    filename="$(echo "$line" | awk '{print $2}')"
    echo "Processing $filename"
    git lfs untrack "$filename"
    git lfs checkout "$filename"
done

# Commit changes
git add --all
git add .gitattributes
git commit -m "Remove LFS tracking for all files"

