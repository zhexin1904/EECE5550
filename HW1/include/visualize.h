// pangolin库
#ifndef _VISUALIZE__H
#define _VISUALIZE__H
#include <pangolin/pangolin.h>
// Eigen库
#include <Eigen/Core>
#include <Eigen/Dense>
 
#include <unistd.h>
 
#include <string>
#include <vector>
#include <fstream>
  
 
void DrawTrajectory(std::vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> poses);
#endif