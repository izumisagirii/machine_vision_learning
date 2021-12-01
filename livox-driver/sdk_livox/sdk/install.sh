if [ -e /opt/livoxlidar ]
then
sudo rm -rf /opt/livoxlidar
fi &&
sudo mkdir /opt/livoxlidar &&
sudo cp -r lib /opt/livoxlidar &&
sudo cp -r include /opt/livoxlidar &&
if [ -e /usr/local/share/livoxlidar ]
then
sudo rm -rf /usr/local/share/livoxlidar
fi &&
sudo cp -r livoxlidar /usr/local/share &&
echo "finish!"

