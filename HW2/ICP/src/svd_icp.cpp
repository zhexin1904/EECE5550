// basic svd-icp

// Zhexin(Jason) Xu , xu.zhex@northeastern.edu
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

pcl::PointCloud<pcl::PointXYZ>::Ptr readData(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream data(path);
    std::stringstream ss;
    std::string line;
    // float x, y, z;
    if (!data.is_open())
    {
        std::cerr << "Can not open dataset !" << std::endl;
    }
    pcl::PointXYZ point;
    while (data >> point.x >> point.y >> point.z) {
        cloudPtr->push_back(point);
    }
    data.close();
    return cloudPtr;
}

Eigen::Vector3d decentralize(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoint_)
{
    Eigen::Vector3d v;
    double x, y, z;
    double k = cloudPoint_->size();
    for (auto it : cloudPoint_->points)
    {
        x += it.x;
        y += it.y;
        z += it.z;
    }
    v << x/k , y/k, z/k;
    return v;
}

Eigen::Vector3d toVec3(pcl::PointXYZ p)
{
    Eigen::Vector3d v;
    double x = p.x;
    double y = p.y;
    double z = p.z;
    v << x, y, z;
    return v;
}

void totxt(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoint, std::string txtPath)
{
    
    std::ofstream file(txtPath);
    if (file.is_open()) {
        for (auto it : cloudPoint->points)
        {
            file << it.x << " " << it.y << " " << it.z << std::endl;
        }
        file.close();
    }
    else{
    std::cerr << "Can not open txt" << std::endl;
    } 
}


// using Brute force nearest serch
// 
int getCorrespondence(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source_, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target_,
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys,
    double min_d
) {
    const float MAX_CORR_DIST_SQR = min_d * min_d * min_d; // 

    size_t num_corr = 0;

    // set up point correspondence
    for(size_t  i =0; i < input_source_->points.size(); ++i)
    {
        int corr_ind = 0;
 
        double min_dis = 1;     
        double current_dis = 0;
        for (size_t j = 0; j < input_target_->points.size(); j++)
        {
            double current_dis = pow((input_source_->points[i].x - input_target_->points[j].x), 2) + 
                            pow((input_source_->points[i].y - input_target_->points[j].y), 2) + 
                            pow((input_source_->points[i].z - input_target_->points[j].z), 2);


            if (current_dis < min_dis )
            {
                min_dis = current_dis;
                corr_ind  = j;
            }
        
        }
        if( min_dis > MAX_CORR_DIST_SQR)
            continue;
            
        // add  correspondence:
        Eigen::Vector3f x(
                    input_target_->at(corr_ind).x,
                    input_target_->at(corr_ind).y,
                    input_target_->at(corr_ind).z
        );
        Eigen::Vector3f y(
                    input_source_->at(i).x,
                    input_source_->at(i).y,
                    input_source_->at(i).z
        );
        xs.push_back(x);
        ys.push_back(y);
        
        ++num_corr;

    }

    std::cout << "number of correspondence :" << xs.size() << std::endl;

    return num_corr;
}


// SVD solution of ICP
// param: corresponding set xs, ys; transform computed 
void getTransform(
    const std::vector<Eigen::Vector3f> &xs,
    const std::vector<Eigen::Vector3f> &ys,
    Eigen::Matrix4f &transformation_
) {
    const size_t N = xs.size();

    // find centroids of mu_x and mu_y:
    Eigen::Vector3f mu_x = Eigen::Vector3f::Zero();
    Eigen::Vector3f mu_y = Eigen::Vector3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        mu_x += xs.at(i);
        mu_y += ys.at(i);
    }
    mu_x /= N; 
    mu_y /= N;

    // build H iteratively
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < N; ++i) {
        H += (ys.at(i) - mu_y) * (xs.at(i) - mu_x).transpose();
    }

    // solve R
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

    // solve t
    Eigen::Vector3f t = mu_x - R * mu_y;

    // set output to Matrix4f using R, t
    transformation_.setIdentity();
    transformation_.block<3, 3>(0, 0) = R;
    transformation_.block<3, 1>(0, 3) = t;
}



Eigen::Matrix4f svdICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr targetPoint, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoint, 
        pcl::PointCloud<pcl::PointXYZ>::Ptr resultPoint, 
        double min_d, double iterations, 
        std::vector<Eigen::Vector3f> xs_corre, 
        std::vector<Eigen::Vector3f> ys_corre
)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPoint_ = targetPoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoint_ = sourcePoint;

    Eigen::Matrix4f init_T;
    init_T.setIdentity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr init_source(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sourcePoint_, *init_source, init_T); // set the initial guess of transform


    Eigen::Matrix4f T_;
    T_.setIdentity();

    for (size_t iteration = 0; iteration < iterations; iteration++)
    {
            
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_source(new  pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*init_source, *current_source, T_); 

        std::vector<Eigen::Vector3f> xs;
        std::vector<Eigen::Vector3f> ys;
        
        if (getCorrespondence(current_source, targetPoint_, xs, ys, min_d) < 2 ) // the number of corre
            break;

        Eigen::Matrix4f delta_transform;

        getTransform(xs, ys, delta_transform);

        T_ = delta_transform * T_;

        // load the correspondce of last iteration(only used for debugger)
        if (iteration == iterations - 1)
        {
                xs_corre = xs;
                ys_corre = ys;
        }

        }
        pcl::transformPointCloud(*sourcePoint_, *resultPoint, T_);
    
    return T_;
}


// load source ad result pointcloud to pcd format together, and adding color to distinguish them
void toPcd(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr result, std::string pcdPath)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  sourceColored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  targetColored(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (auto it : source->points)
    {
        pcl::PointXYZRGB sourcePoint;
        sourcePoint.x = it.x;
        sourcePoint.y = it.y;
        sourcePoint.z = it.z;
        sourcePoint.r = 0;
        sourcePoint.g = 255;
        sourcePoint.b = 0;
        sourceColored->push_back(sourcePoint);
    }
    sourceColored->width = 5;
    sourceColored->height = 5;

    sourceColored->is_dense = true;

        for (auto it : result->points)
    {
        pcl::PointXYZRGB targetPoint;
        targetPoint.x = it.x;
        targetPoint.y = it.y;
        targetPoint.z = it.z;
        targetPoint.r = 255;
        targetPoint.g = 0;
        targetPoint.b = 0;
        targetColored->push_back(targetPoint);
    }
    targetColored->width = 5;
    targetColored->height = 5;

    targetColored->is_dense = true;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *mergedCloud = *sourceColored + *targetColored;

    pcl::io::savePCDFileBinary(pcdPath, *mergedCloud);
    std::cout << "Saved " << mergedCloud->size() << " points to pcd format." << std::endl;
} 





void evaluate(const pcl::PointCloud<pcl::PointXYZ>::Ptr targetPoint, const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoint)
{
    const float MAX_CORR_DIST_SQR = 0.015625;
    std::vector<Eigen::Vector3f> xs;
    std::vector<Eigen::Vector3f> ys;


    for(size_t  i =0; i < targetPoint->points.size(); ++i)
    {
        double corr_ind;

        double min_dis = 1;  
        double current_dis = 0;   
        for (size_t j = 0; j < sourcePoint->points.size(); j++)
        {
            double current_dis = pow((sourcePoint->points[i].x - targetPoint->points[j].x), 2) + 
                            pow((sourcePoint->points[i].y - targetPoint->points[j].y), 2) + 
                            pow((sourcePoint->points[i].z - targetPoint->points[j].z), 2);
            
            if (current_dis < min_dis )
            {
                min_dis = current_dis;
                corr_ind  = j;
            }

        }
        if( min_dis > MAX_CORR_DIST_SQR)
            continue;

        
        Eigen::Vector3f x(
                    targetPoint->at(corr_ind).x,
                    targetPoint->at(corr_ind).y,
                    targetPoint->at(corr_ind).z
        );
        Eigen::Vector3f y(
                    sourcePoint->at(i).x,
                    sourcePoint->at(i).y,
                    sourcePoint->at(i).z
        );
        xs.push_back(x);
        ys.push_back(y);
                
        }

    double error = 0;
    // Eigen::Vector3f errors(Eigen::Vector3f::Zero());
    for (size_t i = 0; i < xs.size(); i++)
    {
        error = error + (ys[i] - xs[i]).norm();   
    }
    
    error = error / ys.size();

    // std::cout << "errors " << std::endl << errors.transpose() << std::endl;
    std::cout << "number of correspondence for RMSE " << xs.size() << std::endl;
    
    std::cout << "RMSE = " << error << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoint, Eigen::Isometry3d T)
{
    for (auto it : cloudPoint->points)
    {
        it.x = (T * toVec3(it))[0];
        it.y = (T * toVec3(it))[1];
        it.z = (T * toVec3(it))[2];
    }
    return cloudPoint;
}

int main(int argc, char ** argv)
{
    double min_d = 0.25;
    double iterations = 30;
    std::string txtPath = "../data/result.txt";
    std::string pcdPath = "../data/comparison_n.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    // if (argc != 3)
    // {
    //     std::cout << "Usage: ./svd_icp sourcedata.txt targetdata.txt" << std::endl;
    //     return 1;
    // }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr source = readData(argv[1]);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr target = readData(argv[2]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source = readData("../data/pclX.txt");
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = readData("../data/pclY.txt");
    std::vector<Eigen::Vector3f> xs_corre;
    std::vector<Eigen::Vector3f> ys_corre;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    Eigen::Matrix4f T = svdICP(target, source, result, min_d, iterations, xs_corre, ys_corre);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    std::cout << "result :" << std::endl << T.matrix() << std::endl;
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    std::cout << "Time used for svd icp: " << time_used.count() << std::endl;

    evaluate(target, result);
    totxt(result, txtPath);

    toPcd(result, target, pcdPath);
    

    return 0;
}