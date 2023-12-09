#include <opencv2/opencv.hpp>
#include <fstream>
#include <vector>

int main() {
    cv::Mat map = cv::imread("/home/jason/EECE5550/hw/HW4/astar/data/occupancy_map.png", cv::IMREAD_COLOR);

    std::ifstream pathFile("/home/jason/EECE5550/hw/HW4/astar/data/path.txt");
    if (!pathFile.is_open()) {
        std::cerr << "Error opening path file." << std::endl;
        return -1;
    }

    std::vector<cv::Point> pathPoints;
    int x, y;
    while (pathFile >> x >> y) {
        pathPoints.emplace_back(y, x);
    }
    pathFile.close();
// cv::Point point1(140, 635);
// cv::Point point2(400, 350);
    for (const cv::Point& point : pathPoints) {
 cv::circle(map, point, 1, cv::Scalar(0, 0, 255), cv::FILLED);    }
// cv::circle(map, point1, 10, cv::Scalar(0, 0, 255), cv::FILLED);
// cv::circle(map, point2, 10, cv::Scalar(0, 0, 255), cv::FILLED);

    cv::imshow("Map with Path", map);
    cv::imwrite("output_image.png", map);
    cv::waitKey(0);

    return 0;
}