#!/bin/bash

# Rename all jpgs in folder given by $1
a=0
for i in $1/*.tif 
do
    new=$(printf "$1/%04d.tif" "$a")
    cp "$i" ${new}
    let a=a+1;
done

