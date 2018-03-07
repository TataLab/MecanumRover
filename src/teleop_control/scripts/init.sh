#!/bin/sh

#echo "initiallizing phase $1"

BASE=$(rospack find teleop_control)/scripts
$BASE/term.sh "$BASE/init_$1.sh" &
