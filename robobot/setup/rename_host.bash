#!/bin/bash
# to be called at boot. 
# Replaces hostname with name in <robotname>.
# Get old and new hostname
hn=`hostname`
# echo "renaming" $hn "again" >/home/local/aaa.txt
# read new name in the file robotname
f="/home/local/svn/log/robotname"
cd /home/local/svn/log
date >> rename_info.txt
if [ -f $f ];
then # file exist
  # get name in file to variable nn
  nn=`cat $f`
  # and length of this name
  echo "found name " $nn
  yy=$(expr length $nn)
  echo "length of" $nn "in" $f "is" $yy "."
  if [ $yy -gt 2 ];
  then # length is longer than 2 characters
    if [ $hn != $nn ]; 
    then # replace old hostname with new
      #echo new name $nn, so rename host from $hn
      echo New name is $nn, so rename host from $hn >> rename_info.txt
      hostnamectl set-hostname $nn
      # sed -i "s/$hn/$nn/g" /etc/hostname
      # seems like /etc/hosts is not updated, but /etc/hostname is
      sed -i "s/$hn/$nn/g" /etc/hosts
    else
      # just add reboot date to file
      echo Same hostname $hn , all is fine. >> rename_info.txt
    fi
  fi
else
  echo "File >" $f "< not found." >> rename_info.txt
fi
