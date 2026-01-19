
sudo apt-get install qtbase5-private-dev libqt5svg5-dev libsdl2-dev libsdl2-image-dev -y
cmake . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  $@
cmake --build build
