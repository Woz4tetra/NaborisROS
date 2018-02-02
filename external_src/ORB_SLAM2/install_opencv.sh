sudo apt-get update
sudo apt-get upgrade
sudo apt-get install cmake git

# opencv
sudo apt-get install build-essential pkg-config
sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng-dev
sudo apt-get install libtbb-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk2.0-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python2.7-dev python3-dev python-numpy python3-numpy

cd ~/Downloads
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.1.0.zip
unzip opencv.zip
cd opencv-3.1.0/
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j3
sudo make install
sudo ldconfig
