// HW1, question1
// by Zhexin(Jason) Xu, (xu.zhex@northeastern.edu)
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
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

int main(int argc, char** argv)
{


    Eigen::Matrix4d O_p1, S_p1, T_SO;
    O_p1 << 2, 0, -1, -1,
        3, 0, -2, 0,
        -3, -3, 2, -2,
        1, 1, 1, 1;

    S_p1 << -1.3840, -0.9608, 1.3250, -1.3140,
            4.5620, 1.3110, -2.3890, 0.2501,
            -0.1280, -1.6280, 1.7020, -0.7620,
            1, 1, 1, 1;

    T_SO = S_p1 * O_p1.inverse();

    cout << "T_SO is :" << endl << T_SO << endl;


    Sophus::SE3d T_SO_SE3;
    T_SO_SE3.so3() = orthogonalize(T_SO.block<3, 3>(0,0));
    T_SO_SE3.translation() = T_SO.block<3, 1>(0,3);
                
    Eigen::Isometry3d T_SO_eigen; 
    T_SO_eigen = S_p1 * O_p1.inverse();

    cout << "T_SO_eigen : " << endl << T_SO_eigen.matrix() <<endl;    

    cout << "T_SO in SE3 is :" << endl << T_SO_SE3.matrix() << endl;


    

    
    return 0;
    
    
}

