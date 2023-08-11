# Carla_opendrive
Separate the code from [Carla](http://carla.org/) for operating OpenDRIVE.


## Building
### Dependencies
Make sure to meet these dependencies in advance:
- [boost_1_70_0](https://www.boost.org/users/history/version_1_70_0.html)  build boost:
```
cd boost_1_70_0
./bootstrap.sh
./b2
sudo ./b2 install
```

### Building on Linux
With CMake as plattform-independent build tool **Carla_opendrive** can be configured for various native build environments. An exemplary configuration for Make under Unix:
```
cd carla_opendrive/carla_opendrive
mkdir build
cd build
cmake ..
```
To build the project afterwards run
```
make
```

# License
This code is distributed under MIT License.
Note that CARLA itself follows its own license terms.
