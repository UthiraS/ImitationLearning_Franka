#!/bin/bash

#iterate over every file in the ycb folder
cur=$(pwd)
pkgPath=$(rospack find active_vision)
src=$pkgPath"/models/ycbAV"


for file in $src/*; do
    if [[ !($file = *.txt) ]] && [[ !($file = *.py) ]] && [[ !($file = */sdf) ]]; then
        echo $file"/google_16k" "output.pcd"
    fi
done