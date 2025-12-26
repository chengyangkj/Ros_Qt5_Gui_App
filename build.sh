
sudo apt-get install qtbase5-private-dev libqt5svg5-dev libsdl-image1.2-dev libsdl1.2-dev -y
cmake . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  $@
cmake --build build
