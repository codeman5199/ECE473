#!/bin/bash
# Bash script to print file to COEPrint
# file is uploaded via scp in ~/print directory
# file is then printed on COEPrint using a2ps
# for best results, set up ssh keys on flip server

# first paramter contains name of file to print. Do not include directory, simply './ printflip examplefile.c:'

# printflip.sh
# Cody McCall
# 10/14/2019

user=""				#insert your username
printer="COEPrint"

if  [ "$#" -ne 1 ]			#check that there is at least one argument passed into the script
then
echo "Error: No arg. No filename provided."
exit 1
fi

filename=$1					#name of file to print
echo "Printing : $filename on $printer..."
scp $pwd$filename $user@access.engr.oregonstate.edu:~/print							#send to flip
ssh $user@access.engr.oregonstate.edu "a2ps -P $printer ~/print/$filename"			#execute a2ps on flip servers
