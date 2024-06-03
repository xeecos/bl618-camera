if [ $1 ]
then
cd ./project
sudo make flash COMX=$1 
fi