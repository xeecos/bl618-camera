if [ $1 ]
then
cd ./project
make flash CPU=bl616 BOARD=bl616dk COMX=$1 
fi