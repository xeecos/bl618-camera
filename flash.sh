if [ $1 ]
then
cd ./project
uNames=`uname -s`
make
osName=${uNames: 0: 4}
if [ "$osName" == "MING" ] # Darwin
then
make flash COMX=$1 
else
sudo make flash COMX=$1
fi
fi