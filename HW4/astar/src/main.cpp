// HW4 Astar of EECE5550
// main 
// zhexin xu (xu.zhex@northeastern.edu)
#include "/home/jason/EECE5550/hw/HW4/astar/include/astar.h"
#include <memory>
int main(int argc, char** argv)
{
    Eigen::Vector2d start(635, 140), goal(350, 400);
    std::string mapPath = "/home/jason/EECE5550/hw/HW4/astar/data/map.txt";
    std::shared_ptr<astarSearch>astarPlanner = std::make_shared<astarSearch>();
    if (astarPlanner->run(start, goal, mapPath)){
    
        auto grid_path = astarPlanner->recoverPath();
        std::ofstream outFile("/home/jason/EECE5550/hw/HW4/astar/data/path.txt");
        if (!outFile.is_open()) {
            std::cerr << "Unable to open the file." << std::endl;
            return 1;
        }

        for (const auto& vec : grid_path) {
            outFile << vec.x() << " " << vec.y() << std::endl;
        }

        outFile.close();
        std::cout << "planning successfully , path length :" << std::endl;
    }
    else{

    std::cout <<"Error: Unable to find the path" << std::endl;

    }

    return 0;
}