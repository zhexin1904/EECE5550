# Introduction

This is the code of the solution in HW1 of EECE5550

# Dependencies

Sophus https://github.com/strasdat/Sophus

Eigen https://eigen.tuxfamily.org/

Pangolin https://github.com/stevenlovegrove/Pangolin

# Build

```bash
cd <yourpath>
git clone https://github.com/zhexin1904/EECE5550.git
cd EECE5550
mkdir build
cmake ..
make
```

# Run example

## question1

```bash'
cd bin
./q1
```

## question3

```bash
cd bin
./q3 <times for intergration>
# for example, trajectory after 30s
./q3 30
```

## question4

```bash
cd bin
# if use SO3 to update ratation
./bin/q4 useSO3==true
# if use quaternion to update ratation
./bin/q4 useSO3==false
```

