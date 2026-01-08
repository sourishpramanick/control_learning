# How to build

cd /home/sourish/mpc_dl/control_learning/build
conan install .. --output-folder=. --build=missing
cmake .. -DCMAKE_PREFIX_PATH=$(pwd)/build/Release/generators -DCMAKE_BUILD_TYPE=Release
make

# How to run
LD_LIBRARY_PATH=/home/sourish/miniconda3/envs/mpc_dl/lib/python3.11/site-packages/casadi:$LD_LIBRARY_PATH \
./control_learning