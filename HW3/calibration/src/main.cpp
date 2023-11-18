// HW3(calibration) of EECE5550
// Zhexin Xu, xu.zhex@northeastern.edu
// 11/11, 2023
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/format.hpp>
#include <string>


 int main(int argc, char **argv)
{
    // std::vector<cv::Mat> grays, Imgs;
    cv::Mat grays, Imgs;
    std::vector<cv::Point2f> corners;
    const cv::Size imgSize(8, 6);
    double error;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    std::vector<std::vector<cv::Point2f>> imagePoints;

    std::vector<cv::Point3f> objectPoint;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 8; ++j) {
            objectPoint.push_back(cv::Point3f(j * 1.f, i * 1.f, 0.0f));
        }
    }

    for (size_t i = 0; i < 8; ++i)
    {
        boost::format fmt("%s/%d.%s"); 
        Imgs = cv::imread((fmt % "data" % i % "JPEG").str(), cv::IMREAD_GRAYSCALE);
        if (cv::findChessboardCorners(Imgs, imgSize, corners)){
            cv::drawChessboardCorners(Imgs, imgSize, corners, true);
            cv::imshow("chessboard", Imgs);
            cv::resizeWindow("chessboard", 800, 600);
            cv::waitKey();
            imagePoints.push_back(corners);
            objectPoints.push_back(objectPoint);

        }
    }
    
    error = cv::calibrateCamera(objectPoints, imagePoints, imgSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    std::cout <<  "re-projection error: " << error << std::endl;    
    std::cout <<  "calibration result:  " << std::endl << cameraMatrix << std::endl;
 
    return 0;
} 