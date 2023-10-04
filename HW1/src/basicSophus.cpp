#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

using namespace std;

int main(int argc, char** argv)
{
// Construct Rotation matrix and quaternion
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1, 0 ,0)).toRotationMatrix();
    Eigen::Quaterniond q(R);
// Construct S03 from Rotation matrix and quaternion
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_q(q);
// Construct so3 for R and q
    Eigen::Vector3d so3_R = SO3_R.log();
    Eigen::Vector3d so3_q = SO3_q.log();
    
    cout << "so3_R = " << endl << so3_R << endl; 
    cout << "so3_q = " << endl << so3_q << endl; 

    Eigen::Vector3d so3_updated(0.01, 0, 0);
// BCH formula 
    // Eigen::Matrix3d R_updated = Sophus::SO3d::exp(so3_updated).matrix() * R;
    
// 
    Eigen::Vector3d p1(2, 2, 2);
    Eigen::Quaterniond p1_to_q(0, p1.x(), p1.y(), p1.z());
    Eigen::Quaterniond q1(Eigen::AngleAxisd(M_PI/3, Eigen::Vector3d(1, 0, 0)));
    Eigen::Quaterniond p2_updated_q = q1*p1_to_q*q1.inverse();
    Eigen::Vector3d p2_updated_1(p2_updated_q.x(), p2_updated_q.y(), p2_updated_q.z());
    
    Eigen::Matrix3d R1(Eigen::AngleAxisd(M_PI/6, Eigen::Vector3d(1, 0, 0)));
    Eigen::Vector3d p2_updated_2 = R1 * p1;

    

    cout << "p2_updated_1 = " << endl << p2_updated_1.transpose() << endl; 
    cout << "p2_updated_2 = " << endl << p2_updated_2.transpose() << endl; 

    Eigen::Matrix3d q1_R(q1);


    Eigen::Vector3d so3_R1 = Sophus::SO3d(q1_R).log(); 
    Eigen::AngleAxisd so3_AngleAxis(so3_R1.norm(), so3_R1.normalized());

    // cout << "so3_AngleAxis = " << endl << so3_AngleAxis.angle() << endl; 


    Eigen::AngleAxisd q_AngleAxis(q1);
    // cout << "so3_AngleAxis = " << endl << q_AngleAxis.angle() << endl; 


    cout << "so3_R1 = " << endl << so3_R1.transpose() << endl; 
    cout << "q_R1 = " << endl << Eigen::Vector3d(q1.x(), q1.y(), q1.z()).transpose() << endl; 

    
    return 0;
    
}
