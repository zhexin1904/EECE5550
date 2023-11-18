# calibration

## Dependencies

OpenCV https://opencv.org/

Eigen https://eigen.tuxfamily.org/

AprilTag https://people.csail.mit.edu/kaess/apriltags/

## build

```bash
git clone XXX
mkdir build
cd build
make ..
make -j
```

## usage

```bash
# back to your PROJECT_SOURCE_DIR
# data path can be changed, but need to modify in line 35-36 of main.cpp:
#        boost::format fmt("%s/%d.%s"); 
#        Imgs = cv::imread((fmt % "data" % i % "JPEG").str(), cv::IMREAD_GRAYSCALE);
cd bin
./calibration
```

# apriltag PnP

## Dependencies

OpenCV https://opencv.org/

Eigen https://eigen.tuxfamily.org/

AprilTag https://people.csail.mit.edu/kaess/apriltags/

## build

```bash
git clone XXX
mkdir build
cd build
make ..
make -j
```

## usage

```bash
# back to your PROJECT_SOURCE_DIR
cd bin
./apriltag_pnp vslam/frame_0.jpg 
```

# vslam

## Dependencies

OpenCV https://opencv.org/

Eigen https://eigen.tuxfamily.org/

AprilTag https://people.csail.mit.edu/kaess/apriltags/

GTSAMhttps://github.com/borglab/gtsam

## build

```bash
git clone XXX
mkdir build
cd build
make ..
make -j
```

## usage

```bash
# back to your PROJECT_SOURCE_DIR
cd bin
./vslam
# time for AprilTag to get estimation may take 0.5 - 0.6 seconds/frame(of course depend on your local machine), so 500 images may takes some time :-)
```

## visualization

```bash
# back to your PROJECT_SOURCE_DIR
cd scripts
python3 plot.py ../data/cameraPose.txt ../data/tagPose.txt 
```

