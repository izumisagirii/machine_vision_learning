if [ -e /opt/livoxlidar ]
then
sudo rm -rf /opt/livoxlidar
fi &&
if [ -e /usr/local/share/livoxlidar ]
then
sudo rm -rf /usr/local/share/livoxlidar
fi &&
echo "finish!"
