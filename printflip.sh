# Bash script to print file to COEPrint
# file is uploaded via scp in ~/print directory
# file is then printed on COEPrint using a2ps

#first paramter contains file to print. Do not incluse directory, simply './ printflip examplefile.c:'

# printflip.sh
# Cody McCall
# 10/14/2019

user="mccallco"				#insert your username
printer="COEPrint"

echo "Printing : $1 on $printer..."
scp $pwd$1 $user@access.engr.oregonstate.edu:~/print
ssh $user@access.engr.oregonstate.edu "a2ps -P $printer ~/print/$1"
