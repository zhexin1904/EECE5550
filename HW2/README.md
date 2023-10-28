# SVD-ICP

## Dependencies

Sophus https://github.com/strasdat/Sophus

Eigen https://eigen.tuxfamily.org/

PCL:https://pointclouds.org/

## build

```bash
#git clone repo in your path
mkdir build
cmake ..
make
```

## usage

Remember to change the txt file path according to your machine.

### Brute force matching ICP

```bash
#under build directory
./../bin/svd_icp
```

### Kdtree-ICP

Kdtree is implemented using pcl. 

```bash
#under build directory
./../bin/kdTree_svd_icp 
```

# Particle Filter

## Dependencies

Sophus https://github.com/strasdat/Sophus

Eigen https://eigen.tuxfamily.org/

## build

```bash
#git clone repo in your path
mkdir build
cmake ..
make
```

## usage

Remember to change the txt file path according to your machine.

```bash
#under build directory
./../bin/SE2_pf
#generate noiseless trajectory to compare
./../bin/SE2_motion
```

plot scripts

```bash
python3 plot_car_trajectory.py ../data/carTrajectory.txt ../data/prediction.txt
python3 plot_car_trajectory.py ../data/carTrajectory.txt ../data/updated.txt
```

