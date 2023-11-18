// HW3(apriltag pnp) of EECE5550
// Zhexin Xu, xu.zhex@northeastern.edu
// 11/11, 2023
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag25h7.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <memory>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <vector>

#include <Eigen/Dense>
// calibration result from Problem1 
/* 
[1462.303953885276, 0, 975.080128980159;
 0, 1467.340596259487, 545.2377080643492;
 0, 0, 1]
 */


// convert gtsam::Pose3 to Eigen::Matrix4f
Eigen::Matrix4f gtsam2transPose(gtsam::Pose3 pose)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix(0,3) = pose.translation().x();
    matrix(1,3) = pose.translation().y();
    matrix(2,3) = pose.translation().z();
    matrix.block<3,3>(0,0) = pose.rotation().matrix().cast<float>();
    return matrix;
}


int main(int argc, char* argv[]) {

    string imgpath=argv[1];
    cv::Mat  image =cv::imread(imgpath, cv::IMREAD_GRAYSCALE);
    
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes36h11;
    auto m_tagDetector = std::make_shared<AprilTags::TagDetector>(m_tagCodes);
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::chrono::duration<double> time_used_extract = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "time_used_extract" << time_used_extract.count() << std::endl;
    
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(1462.303953883514, 1467.340596257327,0,  975.0801289907048, 545.2377080716291));
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise =  gtsam::noiseModel::Isotropic::Sigma(2, 0.1); // one pixel in u and v

    // set measurement of the pnp solver
    vector< gtsam::Point2> measurement;
    for (auto i = 0; i < detections.size(); i++)
    {
        if (detections[i].id == 0)
        {
            measurement.push_back(gtsam::Point2(detections[i].p[0].first, detections[i].p[0].second));
            measurement.push_back(gtsam::Point2(detections[i].p[1].first, detections[i].p[1].second));
            measurement.push_back(gtsam::Point2(detections[i].p[2].first, detections[i].p[2].second));
            measurement.push_back(gtsam::Point2(detections[i].p[3].first, detections[i].p[3].second));

            std::cout << "it.id : " << detections[i].id << std::endl;
            std::cout << "corner0 : " << detections[i].p[0].first << "" << detections[i].p[0].first << std::endl;
            std::cout << "corner1 : " << detections[i].p[1].first << "" << detections[i].p[1].first << std::endl;
            std::cout << "corner2 : " << detections[i].p[2].first << "" << detections[i].p[2].first << std::endl;
            std::cout << "corner3 : " << detections[i].p[3].first << "" << detections[i].p[3].first << std::endl;
            std::cout << "index of id0 in the vector : " << i << std::endl;
        }
    }


    // poses of aprilag in the world frame(origin of the aprilTag)
    vector<gtsam::Point3> points;
    // poses of camera in the world frame(origin of the aprilTag)
    vector<gtsam::Pose3> poses;
    // ground truth of landmarks, according to the tag size and definition of world origin
    gtsam::Point3 p0(-0.005, 0.005, 0); // corner0
    gtsam::Point3 p1(0.005, 0.005, 0); // corner1
    gtsam::Point3 p2(0.005, -0.005, 0); // corner2
    gtsam::Point3 p3(-0.005, -0.005, 0); // corner3

    points.push_back(p0);
    points.push_back(p1);
    points.push_back(p2);
    points.push_back(p3);
    gtsam::Pose3 posePrior(gtsam::Rot3(1, 0, 0, 0, 1, 0, 0, 0, 1), gtsam::Point3(0, 0, -0.01));

    poses.push_back(posePrior);
    // Create a factor graph
    gtsam::NonlinearFactorGraph graph;

    // Add a prior on pose x1. This indirectly specifies where the origin is.
    auto poseNoise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(0.001), gtsam::Vector3::Constant(0.003)).finished());
    
    // graph.push_back(PriorFactor<Pose3>(Symbol('x', 0), poses[0], poseNoise)); // add directly to graph

    for (size_t i = 0; i < poses.size(); ++i) 
    {
      for (size_t j = 0; j < points.size(); ++j) 
      {
      //   SimpleCamera camera(poses[i], *K);
      //   Point2 measurement = camera.project(points[j]);
        graph.push_back(gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>(measurement[j], measurementNoise, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K));
      }
    }
    gtsam::Vector3 mu(0.0001, 0.0001, 0.0001);
    // auto pointNoise = gtsam::noiseModel::Constrained::All(3);
    auto pointNoise = gtsam::noiseModel::Constrained::All(3, mu);
    graph.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 0), points[0], pointNoise)); 
    graph.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 1), points[1], pointNoise)); 
    graph.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 2), points[2], pointNoise)); 
    graph.push_back(gtsam::PriorFactor<gtsam::Point3>(gtsam::Symbol('l', 3), points[3], pointNoise)); 

  graph.print("Factor Graph:\n");


  gtsam::Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i)
    initialEstimate.insert(gtsam::Symbol('x', i), poses[i]);
  for (size_t j = 0; j < points.size(); ++j)
    initialEstimate.insert(gtsam::Symbol('l', j), points[j]);
  initialEstimate.print("Initial Estimates:\n");

// DoglegParams parameters;
gtsam::GaussNewtonParams parameters;
parameters.maxIterations = 100;
parameters.relativeErrorTol = 1e-6;
parameters.setVerbosity("iteration");
// gtsam::GaussNewtonParams::setVerbosity

  // Values result = DoglegOptimizer(graph, initialEstimate, parameters).optimize();
    gtsam::Values result = gtsam::GaussNewtonOptimizer(graph, initialEstimate, parameters).optimize();

    result.print("Final results:\n");
    std::cout << "initial error" << graph.error(initialEstimate) << std::endl;
    std::cout << "final error" << graph.error(result) << std::endl;

    
    


    
  // ************************************************************** 
  // code below is only used for test, not relevent to the solution
  // **************************************************************   

    Eigen::Matrix4d T_AprilTag_cam = detections[4].getRelativeTransform(0.01, 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291);
    Eigen::Matrix3d T_Rpart;
    Eigen::Vector3d T_tpart;
    detections[4].getRelativeTranslationRotation(0.01, 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291, T_tpart, T_Rpart);

    Eigen::Matrix4d T_test;
    T_test.block<3,3>(0,0) = T_Rpart;
    T_test.block<3,1>(0,3) = T_tpart;

    // Eigen::Matrix4d T_AprilTag_world = detections[4].getRelativeTranslationRotation(0.01, 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291);

    std::cout << "T_AprilTag_cam : " << std::endl <<  
        T_AprilTag_cam.matrix() << std::endl;
    Eigen::Matrix4d T_AprilTag_world;
    T_AprilTag_world.block<3,3>(0,0) = T_AprilTag_cam.block<3,3>(0,0).transpose();
    T_AprilTag_world.block<3,1>(0,3) = -T_AprilTag_cam.block<3,3>(0,0).transpose() * T_AprilTag_cam.block<3,1>(0,3);
    T_AprilTag_world(3,3) = 1;
    std::cout << "T_AprilTag_world : " << std::endl <<  
        T_AprilTag_world.matrix() << std::endl;

    std::cout << "T_test : " << std::endl <<  T_test.matrix()
     << std::endl;

    Eigen::Matrix3f k;
    k << 1462.303,  0, 975.080, 0, 1467.312, 545.240,  0, 0, 1;

    gtsam::Pose3 T_ = result.at<gtsam::Pose3>(gtsam::Symbol('x', 0));
    Eigen::Matrix4f T;

    T = gtsam2transPose(T_);

    std::cout << "camera in tag frame :" << std::endl;
    std::cout << T.matrix() << std::endl;
    // std::cout << T.inverse().matrix() << std::endl;

    for (auto it : points)
    {
      Eigen::Vector4f p_c = T * Eigen::Vector4f(it.x(), it.y(), it.z(), 1);
      Eigen::Vector3f p = k * Eigen::Vector3f(p_c[0]/p_c[2], p_c[1]/p_c[2], p_c[2]/p_c[2]);
      std::cout << p.transpose() << std::endl;
    }

  return 0;
}

