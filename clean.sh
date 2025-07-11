#!/bin/bash

local_path=$(dirname $(readlink -f "$0"))

cd $local_path
echo $local_path

rm -rf build install log