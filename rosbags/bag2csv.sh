#!/bin/bash

for topic in `rostopic list -b "$1"` ; do rostopic echo -p -b $1 $topic >$1-bagfile-${topic//\//_}.csv ; done
