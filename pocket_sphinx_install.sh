#! /bin/bash
<<"COMMENT"
A simple shell script to install pocketsphinx and sphinxbase
see: https://wiki.umbc.edu/display/IRAL/PocketSphinx+Install+on+ROS
Also the last steps are:
cd into catkin_ws/src
git clone https://github.com/mikeferguson/pocketsphinx
catkin_make
source devel/setup.bash from catkin_ws
COMMENT
mkdir sphinx
cd sphinx
wget http://sourceforge.net/projects/cmusphinx/files/sphinxbase/5prealpha/sphinxbase-5prealpha.tar.gz
wget http://sourceforge.net/projects/cmusphinx/files/pocketsphinx/5prealpha/pocketsphinx-5prealpha.tar.gz
tar xvf sphinxbase-5prealpha.tar.gz
tar xvf pocketsphinx-5prealpha.tar.gz
sudo apt-get install libpulse-dev
sudo apt-get install bison
cd sphinxbase-5prealpha
make clean
make
sudo make install
cd ../pocketsphinx-5prealpha
./configure
make clean
make
sudo make install
sudo apt-get install gstreamer0.10-pocketsphinx
sudo apt-get install python-gst0.10
sudo apt-get install gstreamer0.10-gconf
