// HW1, question3
// by Zhexin(Jason) Xu, (xu.zhex@northeastern.edu)

#include <iostream>
#include <cmath>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <fstream>
#include "include/visualize.h"
using namespace std;

/* 
The rotation matrix is not orthogonal, which does not meet
the requirement of Sophus in constructing SE3. So we need 
orthogonalization firstly.
 */
Eigen::Matrix3d orthogonalize(const Eigen::Matrix3d& A) 
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
    return R;
}


void questionC(Eigen::Matrix3d R0, Eigen::Matrix3d R1, double t, double dt, Eigen::Vector3d t0, Eigen::Vector3d t1)
{


    Sophus::SE3d SE3_X0(orthogonalize(R0), t0);
    Sophus::SE3d SE3_X1(orthogonalize(R1), t1);

    cout << " SE3_X0" << endl << SE3_X0.matrix() << endl;
    cout << " SE3_X1" << endl << SE3_X1.matrix() << endl;

    Sophus::SE3d SE3_mid;
    SE3_mid = SE3_X0 * Sophus::SE3d::exp(0.5 * ((SE3_X0.inverse() * SE3_X1).log()));
    cout << "midpoint of SE(3) is : " << endl << SE3_mid.matrix() << endl;

    Sophus::SE3d SE3_add;

    string path = "/home/jason/EECE5550/lieGroup/SE3Traj_30s.txt";
    ofstream file(path.c_str());
    // file.open(path.c_str());

    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;

    

    if (!file.is_open()) {
        std::cerr << "check the path of file" << std::endl;
    }
    file << SE3_X0.translation().x() << "," << SE3_X0.translation().y() << "," << SE3_X0.translation().z() << endl;
    while(1)
    {

        SE3_add = SE3_X0 * Sophus::SE3d::exp(dt * ((SE3_X0.inverse() * SE3_X1).log()));
        dt = dt + 0.05;
        file << SE3_add.translation().x() << "," << SE3_add.translation().y() << "," << SE3_add.translation().z() << endl;

        if (dt > t)
           { 
            file.close();
            break;
           }
    
           
        Eigen::Isometry3d T_WB(Eigen::Isometry3d::Identity());
        T_WB.rotate(SE3_add.so3().matrix());
        T_WB.pretranslate(SE3_add.translation());
        
        poses.push_back(T_WB); 
           
           
    }
    // file.close();

    // DrawTrajectory(poses);
    

}

void questionD(Eigen::Matrix3d R0, Eigen::Matrix3d R1, double t, double dt, Eigen::Vector3d t0, Eigen::Vector3d t1)
{

    Sophus::SO3d SO3_X0(orthogonalize(R0)), SO3_X1(orthogonalize(R1));
    
    Sophus::SE3d CurveP_mid;

    CurveP_mid.so3() = SO3_X0 * Sophus::SO3d::exp(0.5 * ((SO3_X0).inverse()*  SO3_X1).log());
    CurveP_mid.translation() = t0 + (0.5 * (-t0 +  t1));
    // SE3_mid = SE3_X0 * Sophus::SE3d::exp(0.5 * ((SE3_X0.inverse() * SE3_X1).log()));
    cout << "midpoint of curve P  under specific multiplication law is: " << endl << CurveP_mid.matrix() << endl;

    Sophus::SE3d CurveP_add;

    string path = "/home/jason/EECE5550/lieGroup/CurveP_30s.txt";
    ofstream file(path.c_str());
    // file.open(path.c_str());

    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
 

    if (!file.is_open()) {
        std::cerr << "check the path of file" << std::endl;
    }
    file << t0.x() << "," << t0.y() << "," << t0.z() << endl;
    while(1)
    {

        CurveP_add.so3() = SO3_X0 * Sophus::SO3d::exp(dt * ((SO3_X0).inverse()*  SO3_X1).log());
        CurveP_add.translation() = t0 + (dt * (-t0 +  t1));
        dt = dt + 0.05;
        file << CurveP_add.translation().x() << "," << CurveP_add.translation().y() << "," << CurveP_add.translation().z() << endl;

        if (dt > t)
           { 
            file.close();
            break;
           }

        Eigen::Isometry3d T_WB(Eigen::Isometry3d::Identity());
        T_WB.rotate(CurveP_add.so3().matrix());
        T_WB.pretranslate(CurveP_add.translation());
        
        poses.push_back(T_WB); 

    }
    
    
    DrawTrajectory(poses);
    
}




int main(int argc, char**argv)
{
    if (argc != 2)
    {
        cout << "please enter the time !" << endl;
        return 1; 
    }
    double t = stod(argv[1]);
    double dt = 0.05;
    Eigen::Matrix3d R0, R1;
    R0 << 0.4330, 0.1768, 0.8839,
        0.2500, 0.9186, -0.3062,
        -0.8660, 0.3536, 0.3536;
    R1 << 0.7500, -0.0474, 0.6597,
        0.4330, 0.7891, -0.4356,
        -0.5000, 0.6124, 0.6124;
        
    Eigen::Vector3d t0(1, 1, 0), t1(2, 4, 3);
    
    questionC(R0, R1, t, dt, t0, t1);
    questionD(R0, R1, t, dt, t0, t1);
    
    return 0;
}