sudo cp cartin.rules /etc/udev/rules.d/
sudo udevadm trigger
sudo udevadm control --reload
sudo service udev restart
