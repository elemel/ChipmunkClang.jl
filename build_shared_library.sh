set -e

mkdir -p Chipmunk2D/build
cd Chipmunk2D/build
cmake .. -G Ninja
ninja
