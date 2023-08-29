sudo cp cartin.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo service udev restart
