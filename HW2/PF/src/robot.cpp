// HW1, question4
// by Zhexin(Jason) Xu, (xu.zhex@northeastern.edu)
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <fstream>
// #include "pangolin/pangolin.h"
#include <unistd.h>
using namespace std;


/* 
set the param of the differential robot:
d, diameter, cm
r，radius of the wheel, cm
left_w, angular velocity of left wheel, rad/s
right_w, augular velocity of right wheel. rad/s
 */
class differentialRobot
{
    public:
        differentialRobot(double d, double r, double right_w, double left_w)
            {
                // the motion model given below is based on the slides of Lecture4
                V_b << 0.5 * r * (right_w + left_w), 0, 0; //linear  velocity of the car
                W_b << 0, 0, r * (right_w - left_w) / d; //angular velocity fo the car
            }
        Eigen::Vector3d V_b, W_b;
};

int main(int argc, char **argv)
{
    double d = 0.5, r = 0.25, right_w = 2, left_w = 1.5;
    double T = 0, dt = 0.05;
    double timeStamp = 1;
    Eigen::Vector3d p0(0, 0, 0);

    
    differentialRobot Car(d, r, right_w, left_w);
    //  (x y z) format, just used to simply visulized by matplotlib 
    string path = "/home/jason/EECE5550/hw/HW2/PF/data/carTrajectory.txt";
    ofstream file(path.c_str());
    if (!file.is_open()) {
        std::cerr << "check the path of file" << std::endl;
    }


    Sophus::SE3d T_wb;
    T_wb.translation() = p0; // initialied with (x0, yo, z0)
    Eigen::Vector3d V_w, P_w;
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    
    while (1)
    {
        timeStamp = timeStamp + 1;
        V_w = T_wb.so3() * Car.V_b;
        P_w = P_w + V_w * dt;
        T_wb.translation() = P_w;

        // update in S03
        
        T_wb.so3() = T_wb.so3() * Sophus::SO3d::exp(Car.W_b * dt);
        

        file << T_wb.translation().x() << "," << T_wb.translation().y() << "," << T_wb.translation().z() << endl;
        

        
        // Poses used to visulized in Pangolin
        Eigen::Isometry3d T_WB(Eigen::Isometry3d::Identity());
        T_WB.rotate(T_wb.so3().matrix());
        T_WB.pretranslate(T_wb.translation() * 0.01); // for visualization better in Pangolin, using a scalar
        poses.push_back(T_WB);
        T  = T + dt;
        if (T > 10)
        {
            file.close();
            break;
        }
    }
        cout << "SE3 of the robot after 5s :" << endl << poses.back().matrix() << endl;
 
    return 0;
}