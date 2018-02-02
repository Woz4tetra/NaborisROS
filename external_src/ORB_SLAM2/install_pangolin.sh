
sudo apt-get install libeigen3-dev libblas-dev liblapack-dev libglew-dev
cd ~/Downloads
git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
mkdir build
cd build
cmake ..
make
