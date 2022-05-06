#!/bin/zsh

gnome-terminal --tab -x zsh -c "\
sudo chmod 777 /dev/xsens; \
roslaunch agx_ws/driver.launch; \
exec zsh"
