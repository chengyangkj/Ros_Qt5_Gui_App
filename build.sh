
sudo apt-get install qtbase5-private-dev libqt5svg5-dev libsdl2-dev libsdl2-image-dev -y
cmake . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$(pwd)/build/install" \
  $@
cmake --build build
cmake --install build