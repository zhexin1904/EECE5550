// HW1, question4
// by Zhexin(Jason) Xu, (xu.zhex@northeastern.edu)
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <fstream>
#include "pangolin/pangolin.h"
#include <unistd.h>
#include "include/visualize.h"
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
    double d = 30, r = 6, right_w = 10, left_w = 8;
    double T = 0, dt = 0.05;
    double timeStamp = 1;
    Eigen::Vector3d p0(0, 0, 0);
    if (argc != 2)
    {
        cout << "Usage: ./bin/q4 useSO3==false" << endl;
        return 1;
    }
    
    bool use_SO3 = argv[1];
    differentialRobot Car(d, r, right_w, left_w);
    //  (x y z) format, just used to simply visulized by matplotlib 
    string path = "/home/jason/EECE5550/lieGroup/carTrajectory.txt";
    ofstream file(path.c_str());
    if (!file.is_open()) {
        std::cerr << "check the path of file" << std::endl;
    }
    // tum format(timestamp x y z q1 q2 q3 q4), but doesn't matter
    string path_tum = "/home/jason/EECE5550/lieGroup/carTrajectory_tum.txt"; 
    ofstream file_tum(path_tum.c_str());
    if (!file_tum.is_open()) {
        std::cerr << "check the path of file" << std::endl;
    }

    file_tum << timeStamp << "" << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 1 << endl;
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
        if (use_SO3 == true)
        {
            T_wb.so3() = T_wb.so3() * Sophus::SO3d::exp(Car.W_b * dt);
        }
        else
        {
        // update in quaternion
            Eigen::Quaterniond q;
            q = T_wb.unit_quaternion() * Eigen::Quaterniond(1, 0.5*Car.W_b(0)*dt, 0.5*Car.W_b(1)*dt, 0.5*Car.W_b(2)*dt);
            q.normalize(); // q needs to normalize everytimes
            T_wb.so3() = Sophus::SO3d(q);
        }
        file << T_wb.translation().x() << "," << T_wb.translation().y() << "," << T_wb.translation().z() << endl;
        
        // convert SO3 to quaternion, for tum format
        Eigen::Quaterniond q(T_wb.so3().matrix());
        q.normalize();
        file_tum << timeStamp << " " << T_wb.translation().x() << " " << T_wb.translation().y() << " " << 3 << " " <<
            q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        timeStamp++;
        
        // Poses used to visulized in Pangolin
        Eigen::Isometry3d T_WB(Eigen::Isometry3d::Identity());
        T_WB.rotate(T_wb.so3().matrix());
        T_WB.pretranslate(T_wb.translation() * 0.01); // for visualization better in Pangolin, using a scalar
        poses.push_back(T_WB);
        T  = T + dt;
        if (T > 5)
        {
            file.close();
            file_tum.close();
            break;
        }
    }
        cout << "SE3 of the robot after 5s :" << endl << poses.back().matrix() << endl;
        DrawTrajectory(poses);
 
    return 0;
}