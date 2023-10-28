// basic svd-icp
// Zhexin(Jason) Xu , xu.zhex@northeastern.edu
// using kd_tree of PCL to do data association
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/flann.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

// reading data from txt,and constructing ptr of pointcloud
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

// geting decentralized coordinates 
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

// converting pcl::PointXYZ to Eigen::Vector3f
Eigen::Vector3f toVec3(pcl::PointXYZ p)
{
    Eigen::Vector3f v;
    double x = p.x;
    double y = p.y;
    double z = p.z;
    v << x, y, z;
    return v;
}

// converting Eigen::Matrix4f to Eigen::Isometry3f, which can use homogeneous transformation directly
Eigen::Isometry3f toTransMatrix(const Eigen::Matrix4f matrix4f)
{
    Eigen::Isometry3f isometry3f = Eigen::Isometry3f::Identity(); 

    isometry3f.linear() = matrix4f.block<3, 3>(0, 0).cast<float>();
    isometry3f.translation() = matrix4f.block<3, 1>(0, 3).cast<float>();

    return isometry3f;
}

// load coordinates in pcl::PointXYZ into txt format
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

// RMSE
// we need to get the corresponse of the last iteration time
// then calculate the RMSE
// here directly using the result of the last itreation
// equal to apply final transformation to source point cloud 
void evaluate(const pcl::PointCloud<pcl::PointXYZ>::Ptr targetPoint, const pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoint)
{
    pcl::search::KdTree<pcl::PointXYZ> kdtree_;   
    const float MAX_CORR_DIST_SQR = 0.015625;
    std::vector<Eigen::Vector3f> xs;
    std::vector<Eigen::Vector3f> ys;

    kdtree_.setInputCloud(targetPoint);
    for(size_t  i =0; i < sourcePoint->points.size(); ++i)
    {
        std::vector<int>  corr_ind;   
        std::vector<float>  corr_sq_dis;      
        kdtree_.nearestKSearch(
                sourcePoint->at(i),
                1,
                corr_ind, corr_sq_dis
        );       
        
        if(corr_sq_dis.at(0) >  MAX_CORR_DIST_SQR)
            continue;
        
        Eigen::Vector3f x(
                    targetPoint->at(corr_ind.at(0)).x,
                    targetPoint->at(corr_ind.at(0)).y,
                    targetPoint->at(corr_ind.at(0)).z
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

// SVD solution of ICP
// param: corresponding set xs, ys; transform computed 
void GetTransform(
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

// find correspondance using kd_tree
// param: source point, target point, correspondig set of source, corresponding set of target, kd_tree ptr 
// return: number of correspondence
int GetCorrespondence(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_source, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_target_,
    std::vector<Eigen::Vector3f> &xs,
    std::vector<Eigen::Vector3f> &ys,
    pcl::search::KdTree<pcl::PointXYZ> kdtree_,
    double min_d
) {
    const float MAX_CORR_DIST_SQR = min_d * min_d * min_d; // 0.25 * 0.25

    size_t num_corr = 0;

    // set up point correspondence
    for(size_t  i =0; i < input_source->points.size(); ++i)
    {
        std::vector<int>  corr_ind;    // index
        std::vector<float>  corr_sq_dis;     // correspondence square distance 
        kdtree_.nearestKSearch(
                input_source->at(i),
                1, // nearest
                corr_ind, corr_sq_dis
        );        
        
        if(corr_sq_dis.at(0) >  MAX_CORR_DIST_SQR)
            continue;
            
        // add  correspondence:
        Eigen::Vector3f x(
                    input_target_->at(corr_ind.at(0)).x,
                    input_target_->at(corr_ind.at(0)).y,
                    input_target_->at(corr_ind.at(0)).z
        );
        Eigen::Vector3f y(
                    input_source->at(i).x,
                    input_source->at(i).y,
                    input_source->at(i).z
        );
        xs.push_back(x);
        ys.push_back(y);
        
        ++num_corr;
    }

    std::cout << "number of correspondence :" << num_corr << std::endl;

    return num_corr;
}

// SVD-ICP workflow
// param: target point, source point, result point, min dis, iterations, corre_target, corre_source
// 1, initialization
// 2, set the inital transform from last time
// 3, find correspondence
// 4, get transform using correspondence
// corre_target, corre_source only used in debugger

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
    pcl::search::KdTree<pcl::PointXYZ> kdtree_;

    Eigen::Matrix4f init_T;
    init_T.setIdentity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr init_source(new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sourcePoint_, *init_source, init_T); // set the initial guess of transform

    kdtree_.setInputCloud(targetPoint_); // set the target point

    Eigen::Matrix4f T_;
    T_.setIdentity();

    for (size_t iteration = 0; iteration < iterations; iteration++)
    {
            
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_source(new  pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*init_source, *current_source, T_); 

        std::vector<Eigen::Vector3f> xs;
        std::vector<Eigen::Vector3f> ys;
        
        if (GetCorrespondence(current_source, targetPoint_, xs, ys, kdtree_, min_d) < 2 ) // the number of corre
            break;

        Eigen::Matrix4f delta_transform;

        GetTransform(xs, ys, delta_transform);

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


// visulization use pcl viewer, can also use  "pcl_viewer xxx.pcd" to visualize
void visulize(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr target, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr result
)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (255, 255, 255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1 (source, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2 (target, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3 (result, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (source, color1, "source");
    viewer->addPointCloud<pcl::PointXYZ> (target, color2, "target");
    viewer->addPointCloud<pcl::PointXYZ> (result, color3, "result");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}



int main(int argc, char ** argv)
{
    double min_d = 0.25;
    double iterations = 30;
    std::string txtPath = "../data/result.txt";
    std::string pcdPath = "../data/comparison.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
    // if (argc != 3)
    // {
    //     std::cout << "Usage: ./svd_icp sourcedata.txt targetdata.txt" << std::endl;
    //     return 1;
    // }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr source = readData(argv[1]);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr target = readData(argv[2]);
    std::vector<Eigen::Vector3f> xs_corre;
    std::vector<Eigen::Vector3f> ys_corre;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source = readData("../data/pclX.txt");
    pcl::PointCloud<pcl::PointXYZ>::Ptr target = readData("../data/pclY.txt");

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    Eigen::Matrix4f T = svdICP(target, source, result, min_d, iterations, xs_corre, ys_corre);
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    std::cout << "result :" << std::endl << T.matrix() << std::endl;
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2);
    std::cout << "Time used for svd icp: " << time_used.count() << std::endl;

    
    evaluate(result, target);

    totxt(result, txtPath);

    toPcd(result, target, pcdPath);
    
    visulize(source, target, result);

    return 0;
}