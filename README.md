

# Prerequisites
Follow this links to find the instructions to install needed software and libraries
- [Bundler Sfm](https://www.cs.cornell.edu/~snavely/bundler/)
- [PMVS](https://www.di.ens.fr/pmvs/)
- [OpenCV](https://docs.opencv.org/3.3.0/d7/d9f/tutorial_linux_install.html)

You can build Bundler and PMVS from source or just download prebuild binaries
# Installing
Clone this repo and set `BUNDLER_PATH` and `PMVS_PATH` inside the `reconstruct.sh` script to your Bundler and PMVS pathes correspondingly.
Create a `build` directory and build the programm.
```
mkdir build
cd build
cmake ..
make 
``` 
# Usage
Place your JPG images in a seperate folder and run the `reconstructon.sh` script inside that folder. The output will be places in the `result.ply` file.

If you already have a point cloud and you only want to detect planes in it run the `plane_detection` binary which is in the `build` folder.
> ./plane_detection *path_to_input_file.ply* *path_to_output_file.ply* 