// HW1, question4
// by Zhexin(Jason) Xu, (xu.zhex@northeastern.edu)
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <sophus/so3.hpp>
#include <fstream>
// #include "pangolin/pangolin.h"
#include <unistd.h>
#include <random>
// #include "include/visualize.h"
// using namespace std;


/* 
set the param of the differential robot:
d, diameter, cm
r，radius of the wheel, cm
left_w, angular velocity of left wheel, rad/s
right_w, augular velocity of right wheel. rad/s
 */
class differentialRobotSE2
{
    public:
        differentialRobotSE2(double d, double r, double right_w, double left_w, double sigma_left, double sigma_right)
            {
                this->sigma_left = sigma_left;
                this->sigma_right = sigma_right;
                this->right_w = right_w;
                this->left_w = left_w;
                this->r = r;
                this->d = d;
            }
            // void addNoise(double sigma_right, double sigma_left)
            // {
            //     V_b << 0.5 * r * ((right_w + sigma_right) + (left_w + sigma_left)), 0;; //linear  velocity of the car
            //     angle =  r * ((right_w + sigma_right) - (left_w + sigma_left)) / d;
            // }
        Eigen::Vector2d V_b;
        double angle;
        double sigma_left;
        double sigma_right;
        double right_w, left_w;
        double d, r;
};



class measurement
{
    public:
    measurement(std::vector<Eigen::Vector2d> positions, double sigma) 
        {
            this->positions = positions;
            // this->nosie << sigma * sigma, 0, 
            //                 0, sigma * sigma;
            this->sigma = sigma;
        }
            
    double weight(Eigen::Vector2d pre, int time, double sigma_p)
    {
        Eigen::Matrix2d nosie;
        nosie << sigma_p * sigma_p, 0, 
                            0, sigma_p * sigma_p;
        double exponent = -0.5 * (pre - positions[time]).transpose() * nosie.inverse() * (pre - positions[time]);
        // double exponent = 1 / ((pre - positions[time]).transpose() * nosie.inverse() * (pre - positions[time]));
        double coefficient = 1 / (2 * M_PI * std::sqrt(nosie.determinant()));

    return coefficient * std::exp(exponent);
        // return exponent;
        
    }
    std::vector<Eigen::Vector2d> positions;
    // Eigen::Matrix2d nosie;
    double sigma;
};



struct particle
{
    int id;
    Sophus::SE2d pose;
    double weight;
};

// init pf
std::vector<particle> pf_init(int num, int x_, int y_, int angle_, std::vector<particle> partciles, std::ofstream &prediction_data)
{


    partciles.resize(num);
    std::default_random_engine generator;
    std::normal_distribution<double> disribution_x(x_, 0.0001);
    std::normal_distribution<double> disribution_y(y_, 0.0001);
    std::normal_distribution<double> disribution_angle(angle_, 0.0001);

    for (int i = 0; i < num; i++)
    {
        double x, y, angle;
        x = disribution_x(generator);
        y = disribution_y(generator);
        angle = disribution_angle(generator);
        angle = M_PI / angle;


        // assuming absolutly konwn of the initial poses, don't use normal_distribution, but keep that part code
        Sophus::SE2d pose_;
        pose_.rot(0.0); 
        partciles.at(i).id = i;
        partciles.at(i).pose = pose_;
        partciles.at(i).weight = 1.0;
        // prediction_data << pose_.translation().x() << "," << pose_.translation().y() << std::endl;

    }
    return partciles;
        
}

// generate zero Guassian noise according to the sigma
std::vector<double> genNoise(double sigma)
{
    std::random_device rd;
    std::mt19937 gen(rd()); 

    std::normal_distribution<double> distribution(0.0, sigma); // Gaussian

    std::vector<double> gaussian_samples;

    for (size_t i = 0; i < 1000; ++i) {
        double sample = distribution(gen); 
        gaussian_samples.push_back(sample); 
    }

    return gaussian_samples;
}

// pf propogation, using motion model
std::vector<particle> pf_propagation(double t1, double t2, differentialRobotSE2 Car, std::vector<particle> particles_t1, std::ofstream &prediction_data)
{

    std::vector<double> sigma_l = genNoise(Car.sigma_left);
    std::vector<double> sigma_r = genNoise(Car.sigma_right);

    double T = t2 - t1;

    double dt = 0.05;
    std::vector<particle> particle_pre;
    
    for (size_t i = 0; i < particles_t1.size(); i++)
    {
        Eigen::Vector2d V_w(0, 0);
        Eigen::Vector2d P_w(0, 0);
        // Sophus::SE2d T_wb_1;
        Sophus::SE2d T_wb_2;
        T_wb_2 = particles_t1[i].pose;

        Eigen::Vector2d V_b;
        double angle;
        double counter = 0;
        V_b << 0.5 * Car.r * ((Car.right_w + sigma_l[i]) + (Car.left_w + sigma_l[i])), 0;; //linear  velocity of the car
        angle =  Car.r * ((Car.right_w + sigma_r[i]) - (Car.left_w + sigma_l[i])) / Car.d;
        while (counter < T)
        {
        
            V_w = T_wb_2.so2() * V_b;
            P_w =  T_wb_2.translation() + V_w * dt;
            // update in translation
            T_wb_2.translation() = P_w;
            // update in SO2


            T_wb_2.so2() = T_wb_2.so2() * Sophus::SO2d::exp(angle * dt);
            counter = counter + dt;

            }

            prediction_data << T_wb_2.translation().x() << "," << T_wb_2.translation().y() << std::endl;
            particles_t1[i].pose = T_wb_2;



    }

    prediction_data << "*" << std::endl;


    return particles_t1;
}





// different methods of resampling
std::vector<particle> resampling_1(std::vector<particle> particles_, std::vector<double> weights)
{
    std::vector<particle> updateParticles(particles_.size());
    std::random_device generator;
    std::mt19937 gen(generator());
    
    std::discrete_distribution<> distribution{std::begin(weights), std::end(weights)}; 

    for (size_t i = 0; i < particles_.size(); i++)
    {
        updateParticles[i] = particles_[distribution(generator)];
        
    }
    
    
    return updateParticles;
}

// different methods of resampling
std::vector<particle>  resampling_2(std::vector<particle> particles_, std::vector<double> weights)
{
std::vector<particle> resampledParticles;

  std::vector<double> cumulativeWeights;
double cumulativeWeight = 0.0;
for (const particle& particle : particles_) 
{
    cumulativeWeight += particle.weight;
    cumulativeWeights.push_back(cumulativeWeight);

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    double startPoint = distribution(generator);

    int index = 0;
    double step = cumulativeWeight / particles_.size();
    double cumulative = startPoint;

    for (int i = 0; i < particles_.size(); ++i) {
        while (cumulative > cumulativeWeights[index]) {
            index++;
        }
        resampledParticles.push_back(particles_[index]);
        cumulative += step;
    }


}  
    
    
    return resampledParticles;
}

// different methods of resampling
std::vector<particle> resample_3(const std::vector<particle>& particles) {
    std::vector<particle> resampled_particles;
    std::vector<double> cumulative_weights;

    double cumulative_weight = 0.0;
    for (const particle& particle : particles) {
        cumulative_weight += particle.weight;
        cumulative_weights.push_back(cumulative_weight);
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    // resample
    for (size_t i = 0; i < particles.size(); ++i) {
        double random_num = dis(gen);

        size_t index = 0;
        while (index < cumulative_weights.size() && cumulative_weights[index] < random_num) {
            index++;
        }

        resampled_particles.push_back(particles[index]);
    }

    return resampled_particles;
}




// pf update
// 1, calculate weight, using Guassian pdf
// 2, resampling according to weight
std::vector<particle> pf_updata(std::vector<particle> particles, measurement GPS, double time, std::ofstream &updated_data)
{
    std::vector<double> weights;
    std::vector<double> weights_normalize;
    std::vector<particle> updateParticles;
    std::vector<double> sigma_r = genNoise(GPS.sigma);
    double totalWeight = 0;
    for ( auto it : particles)
    {

        double weight = GPS.weight(it.pose.translation(), time, sigma_r[it.id]);
        weights.push_back(weight);
        // it.weight = weight;
        totalWeight = weight + totalWeight;
    }
    weights_normalize.resize(weights.size());

    for (size_t i = 0; i < weights.size(); i++)
    {
        weights_normalize[i] = weights[i] / totalWeight;
        particles[i].weight = weights_normalize[i];
    }
        
    updateParticles = resample_3(particles);

    for (auto it : updateParticles)
    {
        updated_data << it.pose.translation().x() << "," << it.pose.translation().y() << std::endl;
    }
    updated_data << "*" << std::endl;


    return updateParticles;
}





int main(int argc, char **argv)
{
    double d = 0.5, r = 0.25, right_w = 2, left_w = 1.5;
    double sigma_l = 0.05, sigma_r = 0.05, sigma_p = 0.10;

    std::vector<Eigen::Vector2d> positions;
    positions.push_back(Eigen::Vector2d(1.6561, 1.2847));
    positions.push_back(Eigen::Vector2d(1.0505, 3.1059));
    positions.push_back(Eigen::Vector2d(-0.9875, 3.2118));
    positions.push_back(Eigen::Vector2d(-1.6450, 1.1978));


    std::string prediction_path = "/home/jason/EECE5550/hw/HW2/PF/data/prediction.txt";
    std::ofstream pre_file(prediction_path.c_str());
    if (!pre_file.is_open()) {
        std::cerr << "check the path of prediction path" << std::endl;
    }
    std::string updated_path = "/home/jason/EECE5550/hw/HW2/PF/data/updated.txt";
    std::ofstream update_file(updated_path.c_str());
    if (!update_file.is_open()) {
        std::cerr << "check the path of updated path" << std::endl;
    }

    std::vector<particle> particles;
    differentialRobotSE2 Car(d, r, right_w, left_w, sigma_l, sigma_r);
    measurement GPS(positions, sigma_p);
    particles = pf_init(1000, 0, 0, 0, particles, pre_file);

    // 0
    particles = pf_propagation(0, 5, Car, particles, pre_file);
    particles = pf_updata(particles, GPS, 0, update_file);

    // 1
    particles = pf_propagation(5, 10, Car, particles, pre_file);
    particles = pf_updata(particles, GPS, 1, update_file);

    // 2
    particles = pf_propagation(10, 15, Car, particles, pre_file);
    particles = pf_updata(particles, GPS, 2, update_file);

    // 3
    particles = pf_propagation(15, 20, Car, particles, pre_file);
    particles = pf_updata(particles, GPS, 3, update_file);

    pre_file.close();
    update_file.close();
    
    
    
 
    return 0;
}