# TRAXXS

A framework to create and manage trajectories.

## Dependencies

* Eigen 3
* [softMotion](https://git.openrobots.org/robots/softmotion/soft-motion-libs.git) (optional, shipped, as submodule)
* [scurve_traj_gen](https://github.com/JimmyDaSilva/scurve_traj_gen) (optional, shipped, as submodule)

### Dependencies installation

#### Eigen 3

```bash
sudo apt-get install libeigen3-dev
```

## Installation

```bash
# clone the repo and go the the root folder
git clone [...]
# update and initialize the submodules
git submodule update --init --recursive
# create a build directory
mkdir build && cd build
# build it
cmake ..
make
# install
sudo make install
```

### Build options

* `USE_TG_SOFTMOTION`, defaults to `ON` : whether or not to use the softmotion implementation
* `USE_TG_SCURVETRAJGEN`, defaults to `ON` : whether or not to use the scurve_traj_gen implementation
* `BUILD_SAMPLES`, defaults to `ON` : whether or not to build the samples

## Use the examples


Some samples output a json describing the trajectory. A python plotter is provided to interpret this output.    
Example usage:    
```bash
## Go to your build directory.    
# samples for path creation
./samples/sample_path
./samples/sample_path_cart
# samples for raw trajectory
./samples/sample_traj > out.json && python samples/python/plot_json.py out.json
./samples/sample_traj_cart > out.json && python samples/python/plot_json.py out.json
# samples for trajectory tracking
./samples/sample_tracker_time_cart > out.json && python samples/python/plot_json.py out.json
./samples/sample_tracker_space_cart > out.json && python samples/python/plot_json.py out.json
```


## ToDos

* Check consistency of use of const reference passing of `shared_ptr`
