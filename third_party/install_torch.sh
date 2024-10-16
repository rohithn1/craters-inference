#!/bin/bash

# Google Drive file ID and file name
FILE_ID="1TqC6_2cwqiYacjoLhLgrZoap6-sVL2sd"
FILE_NAME="torch-1.10.0a0+git36449ea-cp36-cp36m-linux_aarch64.whl"

# Step 1: Use wget to get the confirmation token and download the file
CONFIRM=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate \
    "https://drive.google.com/uc?export=download&id=${FILE_ID}" -O- \
    | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1/p')

# Step 2: Download the file with the confirmation token
wget --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=${CONFIRM}&id=${FILE_ID}" \
    -O "${FILE_NAME}" && rm -rf /tmp/cookies.txt

# Step 3: Check if the file was downloaded successfully
if [ -f "${FILE_NAME}" ]; then
    echo "File downloaded successfully: ${FILE_NAME}"

    # Step 4: Install the downloaded .whl file using pip
    sudo -H python3 -m pip install "${FILE_NAME}"
else
    echo "Failed to download the file."
fi
