#!/bin/sh

#script for launching terminal in places
#disables write acess (only output)

gnome-terminal --tab -e "bash -c '$1'"
#; exec bash
