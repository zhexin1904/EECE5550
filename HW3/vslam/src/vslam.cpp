// HW3(apriltag pnp) of EECE5550
// Zhexin Xu, xu.zhex@northeastern.edu
// 11/11, 2023
// Many commented-out lines of code are merely used for testing purposes 
// and are not relevant to the implementation.

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
#include <boost/format.hpp>
#include <fstream>
#include <chrono>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

/* 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291
[1462.303953883514, 0, 975.0801289907048;
 0, 1467.340596257327, 545.2377080716291;
 0, 0, 1]
 */

// Eigen::Matrix4d to gtsam::Pose3
// Apritag detection I used will return Eigen::Matrix4d
gtsam::Pose3 trans2gtsamPose(Eigen::Matrix4d matrix_)
{
    gtsam::Rot3 rot3(matrix_(0,0), matrix_(0,1), matrix_(0,2),\
                     matrix_(1,0), matrix_(1,1), matrix_(1,2),\
                     matrix_(2,0), matrix_(2,1), matrix_(2,2));
    return gtsam::Pose3(rot3, gtsam::Point3(matrix_(0,3), matrix_(1,3), matrix_(2,3)));
}


int main(int argc, char **argv)
{
    vector<cv::Mat> Imgs;
    for (size_t i = 0; i < 500; i++)
    {
            // data initialize
            cv::Mat Img;
            boost::format fmt("%s/%s%d.%s"); 
            Img = cv::imread((fmt % "vslamData" % "frame_" %i % "jpg").str(), cv::IMREAD_GRAYSCALE);
            Imgs.push_back(Img);
    }

    std::cout << "Imgs.size() : " <<Imgs.size() << std::endl;
   
    gtsam::NonlinearFactorGraph graph;
    gtsam::noiseModel::Isotropic::shared_ptr betweenNoise = gtsam::noiseModel::Isotropic::Sigma(6, 0.001); // pose3 noise
    gtsam::noiseModel::Constrained::shared_ptr landmarkPriorNoise = gtsam::noiseModel::Constrained::All(6, 0.0001);
    gtsam::Pose3 landmarkPrior(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));
    graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('l', 0), landmarkPrior, landmarkPriorNoise));

    gtsam::Values initialEstimate;

    AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes36h11;
    auto m_tagDetector = std::make_shared<AprilTags::TagDetector>(m_tagCodes);
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (auto i = 0; i < Imgs.size(); i++)
    {
        // based on frame
        cv::Mat image = Imgs[i];
        cv::Mat rotatedImage, imagecopy;
        image.copyTo(imagecopy);

		// m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
		vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image);
        gtsam::Symbol pose('x', i);
        Eigen::Matrix4d pose_ ;
        // initialEstimate.insert(pose, )
        for (size_t j = 0; j < detections.size(); j++)
        {
            gtsam::Symbol landmark('l', detections[j].id);
            gtsam::Pose3 T;
            // 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291
            // pose_ = detections[j].getRelativeTransform(0.01, 1467.312, 1462.276, 545.240,  943.933).inverse(); 
            // Eigen::Matrix3d r;
            // Eigen::Vector3d t;
            // detections[j].getRelativeTranslationRotation(0.01, 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291, t, r);
            // Eigen::Matrix4d T_cam2tag; // in object frame(x forward, y left, z up)
            // T_cam2tag.block<3,3>(0,0) = r.transpose();
            // T_cam2tag.block<3,1>(0,3) =  t;
            // T_cam2tag.block<1,3>(3,0) = Eigen::Vector3d(0,0,0).transpose();
            // T_cam2tag(3,3) = 1;

            // converting from camera frame (z forward, x right, y down) to
            // object frame (x forward, y left, z up)
            // Eigen::Matrix4d M;
            // M <<
            //     0,  0, 1, 0,
            //     1, 0, 0, 0,
            //     0, -1, 0, 0,
            //     0,  0, 0, 1;

            // std::cout << "pose_" << pose_.matrix() << std::endl;
            pose_ = detections[j].getRelativeTransform(0.01, 1462.303953883514, 1467.340596257327, 975.0801289907048, 545.2377080716291); 
            T = trans2gtsamPose(pose_);
            // T = trans2gtsamPose(pose_);

            graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(pose, landmark, T, betweenNoise));
        }
        // set the initial estimate of camera pose
        initialEstimate.insert(pose, trans2gtsamPose(pose_));
        // initialEstimate.insert(pose, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)));
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used_extract = chrono::duration_cast<chrono::duration<double>>(t2 - t1);


    // set the initial estimate of apriltag, according to the physical size (just a random guess)
    // this will not affect the optimization , initial guess can be set really randomly
    initialEstimate.insert(gtsam::Symbol('l', 0), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 1), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(1.5e-2, 0, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 2), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(3e-2, 0, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 3), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(4.5e-2, 0, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 4), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(6e-2, 0, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 5), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(7.5e-2, 0, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 6), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 1.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 7), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(1.5e-2, 1.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 8), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(3e-2, 1.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 9), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(4.5e-2, 1.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 10), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(6e-2, 1.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 11), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(7.5e-2, 1.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 12), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 3e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 13), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(1.5e-2, 3e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 14), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(3e-2, 3e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 15), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(4.5e-2, 3e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 16), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(6e-2, 3e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 17), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(7.5e-2, 3e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 18), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(0, 4.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 19), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(1.5e-2, 4.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 20), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(3e-2, 4.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 21), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(4.5e-2, 4.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 22), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(6e-2, 4.5e-2, 0)));
    initialEstimate.insert(gtsam::Symbol('l', 23), gtsam::Pose3(gtsam::Rot3::identity(), gtsam::Point3(7.5e-2, 4.5e-2, 0)));

    
    graph.print("Factor Graph:\n");

    gtsam::LevenbergMarquardtParams LMparams;
    // params.absoluteErrorTol = 1e-6;
    // LMparams.maxIterations = 5;
    gtsam::Values result;

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, LMparams);
    
    chrono::steady_clock::time_point t3 = chrono::steady_clock::now();
    // result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate, LMparams).optimize();
    result = optimizer.optimize();

    chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
    chrono::duration<double> time_used_optimization = chrono::duration_cast<chrono::duration<double>>(t4 - t3);
        

    // result.print("Final results:\n");
    std::cout << "unnormalized initial error" << graph.error(initialEstimate) << std::endl;
    std::cout << "unnormalized final error" << graph.error(result) << std::endl;

    
    string cameraPath = "/home/jason/EECE5550/hw/HW3/vslam/data/cameraPose.txt";
    ofstream camera_file(cameraPath.c_str());
    if (!camera_file.is_open()) {
        std::cerr << "check the path of camera pose path" << std::endl;
    }
    
    string tagPath = "/home/jason/EECE5550/hw/HW3/vslam/data/tagPose.txt";
    ofstream tag_file(tagPath.c_str());
    if (!tag_file.is_open()) {
        std::cerr << "check the path of april tag pose path" << std::endl;
    }
    std::cout << "result size" << result.size() << std::endl;
    for (const auto& key_value : result){
        // gtsam::Pose3 cam = result.at<gtsam::Pose3>(gtsam::Symbol('x', ))
        gtsam::Key key = key_value.key;
        if (gtsam::Symbol(key).chr() == 'x'){
        // 6 DOF poses of camera
        gtsam::Quaternion q = result.at<gtsam::Pose3>(key).rotation().toQuaternion();
        camera_file << gtsam::Symbol(key).index() << "," << result.at<gtsam::Pose3>(key).x() << "," << result.at<gtsam::Pose3>(key).y() << 
        "," << result.at<gtsam::Pose3>(key).z() << "," << q.x() << "," <<q.y() << "," << q.z() << "," << q.w() << std::endl;

        }
        else{
        tag_file << gtsam::Symbol(key).index() << "," <<result.at<gtsam::Pose3>(key).x() << "," << result.at<gtsam::Pose3>(key).y() << 
        "," << result.at<gtsam::Pose3>(key).z() << std::endl;
        }

    }
    std::cout << "time_used for gtsam apriltag" << time_used_extract.count() << std::endl;
    std::cout << "time_used for gtsam optimization" << time_used_optimization.count() << std::endl;


       return 0;
   
   
}



