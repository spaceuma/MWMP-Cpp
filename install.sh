sudo apt-get -y install libopencv-dev cmake
rm -rf build
mkdir build
cd build
cmake -Wno-dev ..
cmake --build ./
