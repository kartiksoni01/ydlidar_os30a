export EYS3D_HOME=./eYs3D/
echo $EYS3D_HOME

rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RELEASE -DESPDI_PLATFORM_NAME=X86_64 ..
make install

