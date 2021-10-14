sudo apt-get -y install libopencv-dev cmake
sudo apt install libgtest-dev build-essential cmake

cur=$PWD
cd /usr/src/googletest
sudo cmake .
sudo cmake --build . --target install

cd $cur
rm -rf build
mkdir build
cd build
cmake -Wno-dev ..
cmake --build ./

mv runUnitTests ../test/unit/
