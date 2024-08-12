export EYS3D_HOME=./eYs3D/
echo $EYS3D_HOME

rm -rf build_NVIDIA
mkdir build_NVIDIA
cd build_NVIDIA
cmake -DCMAKE_BUILD_TYPE=RELEASE .. -DESPDI_PLATFORM_NAME=NVIDIA_64
make install

