#!/bin/bash
# echo -e "\nStarting mission\n"
cd /home/local/Idefix/robobot/raubase/build
./raubase >log_out.txt 2>log_err.txt &
# echo "mission ended"
exit 0
