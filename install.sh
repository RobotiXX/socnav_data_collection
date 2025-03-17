cd ros2_ws/src/ds4drv
mkdir -p ~/.local/lib/python3.10/site-packages
python3 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
