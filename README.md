# object_detection

Setup:

- Run `git submodule update --init --recursive`
- `mkdir build`
- `cd build`
- `make`

To run kinect driver:
  - cd libfreenect
  - `mkdir build`
  - `cd build`
  - `make`
  -  run build/bin/Protonect
  (this will publish lcm depth image types)
  
To run PointCloud Factory:
  - `mkdir build`
  - `cd build`
  - `make`
  -  run kinectPointCloud 
  (this will subscribe to lcm depth images and publish filtered point cloud lcm messages)
