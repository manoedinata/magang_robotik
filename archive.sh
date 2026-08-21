#!/bin/bash

# Check if a repository URL was provided
if [ -z "$1" ]; then
  echo "Usage: ./clone_and_zip.sh <repository_url>"
  exit 1
fi

REPO_URL=$1
REPO_NAME=$(basename "$REPO_URL" .git)
ZIP_FILE="${REPO_NAME}.zip"

echo "Step 1: Cloning repository..."
git clone "$REPO_URL" "$REPO_NAME"

# Check if the clone was successful
if [ $? -eq 0 ]; then
  echo "Step 2: Removing the .git folder..."
  # This deletes the Git tracking folder, leaving just the project files
  rm -rf "$REPO_NAME/.git"
  
  echo "Step 3: Zipping the directory..."
  zip -r "$ZIP_FILE" "$REPO_NAME"
  
  if [ $? -eq 0 ]; then
    echo "Done! Created $ZIP_FILE and kept the clean directory."
  else
    echo "Error: Failed to create the ZIP file."
    exit 1
  fi
else
  echo "Error: Failed to clone the repository."
  exit 1
fi
