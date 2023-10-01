#!/bin/bash

if [ ! $1 ]; then
    echo "Give target folder..."
    exit 1
fi

TARGET_FOLDER=${1}/ELRS
SOURCE_FOLDER=".pio/build/LOGGER_for_HDZero_Goggle"

if [ ! -f "${SOURCE_FOLDER}/backpack.bin" ]; then
    echo "Please build binaries first..."
    exit 1
fi


# Check that target folder esits
if [ ! -d "$TARGET_FOLDER" ]; then
    mkdir $TARGET_FOLDER
fi

# Copy binaries
cp "${SOURCE_FOLDER}/firmware.bin" $TARGET_FOLDER/
cp "${SOURCE_FOLDER}/boot_app0.bin" $TARGET_FOLDER/
cp "${SOURCE_FOLDER}/bootloader.bin" $TARGET_FOLDER/
cp "${SOURCE_FOLDER}/partitions.bin" $TARGET_FOLDER/
