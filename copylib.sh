#!/bin/bash

# Source and destination paths (edit these as needed)
SRC_PATH="Android/app/build/intermediates/cxx/Debug/5532b314/obj/arm64-v8a/libapriltagnative.so"
DEST_DIR="Unity/Assets/Plugins/Android/libs/arm64-v8a/libapriltagnative.so"

# Check if source file exists
if [ ! -f "$SRC_PATH" ]; then
	  echo "Error: Source file does not exist at $SRC_PATH"
	    exit 1
fi

cp -v "$SRC_PATH" "$DEST_DIR"

# Confirm the copy was successful
if [ $? -eq 0 ]; then
	  echo "Successfully copied libapriltagnative.so to $DEST_DIR"
  else
	    echo "Failed to copy the file."
	      exit 1
fi
