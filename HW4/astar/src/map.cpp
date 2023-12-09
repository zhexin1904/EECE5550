// HW4 Astar of EECE5550
// process map from png 
// zhexin xu (xu.zhex@northeastern.edu)
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

int main(int argc, char** argv) 
{
    std::string mapPath = argv[1];
    if (argc != 2)
    {
        std::cerr << ("Usage : ./mapProcess path_to_map.png !");
        return -1;
    }
    
    cv::Mat image = cv::imread(mapPath, cv::IMREAD_GRAYSCALE);

    if (image.empty()) {
        std::cerr << "Error: Unable to read the image." << std::endl;
        return -1;
    }

    std::vector<std::vector<int>> binaryArray;
    // rows/height
    for (int i = 0; i < image.rows; ++i) {
        std::vector<int> row;
        for (int j = 0; j < image.cols; ++j) {
            row.push_back(image.at<uchar>(i, j) == 0 ? 1 : 0); 
        }
        binaryArray.push_back(row);
    }

    std::ofstream outFile("/home/jason/EECE5550/hw/HW4/astar/data/map.txt");

    if (!outFile.is_open()) {
        std::cerr << "Error: Unable to open the output file, chech the path !." << std::endl;
        return -1;
    }

    for (const auto& row : binaryArray) {
        for (int value : row) {
            outFile << value << " ";
        }
        outFile << std::endl;
    }

    std::cout << "Occuapncy_map has been saved to map.txt" << std::endl;

    return 0;
}
