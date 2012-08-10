# install X
sudo apt-get update
apt-get dist-upgrade
sudo apt-get install -y xserver-xorg xserver-xorg-core lightdm x11-xserver-utils mesa-utils pciutils lsof

# add user
man adduser
adduser rosbuild --gecos ""

# setup auto xsession login
sudo echo "
[SeatDefaults]
greeter-session=unity-greeter
autologin-user=rosbuild
autologin-user-timeout=0
user-session=ubuntu
" > /etc/lightdm/lightdm.conf
initctl stop lightdm 
initctl start lightdm 

su - rosbuild

# install ros
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
apt-get install ros-fuerte-desktop-full 



