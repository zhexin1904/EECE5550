// only used for debug in VScode !!!!!!!!!!!!!

#include <iostream>
#include <Eigen/Eigen>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <memory>

#define inf 1>>20
struct node;

typedef node* nodePtr;
typedef std::vector<std::vector<int>> gridMap;
struct node
{
    int id;        // 1--> open set, -1 --> closed set, 0 --> neither open nor close set
    Eigen::Vector2d coord; 
    Eigen::Vector2i index;
    int coord_row, coord_col;
    
    double gScore, fScore;
    nodePtr parentNodePtr;
    // std::multimap<double, nodePtr>::iterator nodeMapIt;

    node(Eigen::Vector2i _index, Eigen::Vector2d _coord){  
      id = 0;
      index = _index;
      coord = _coord;
      gScore = inf;
      fScore = inf;
      parentNodePtr = NULL;
    }

      node(int coord_row_, int coord_col_){  
      id = 0;
      coord_row = coord_row_;
      coord_col = coord_col_;
      gScore = inf;
      fScore = inf;
      parentNodePtr = NULL;
    }
    
    node(){};
    ~node(){};
    
};
class astarSearch
{
    private:

    protected:
        gridMap astarMap;
        int row_SIZE = 680, col_SIZE = 623;
        nodePtr goalPtr;

        std::multimap<double, nodePtr> openSet;

        nodePtr nodeMapPtr[680][623];

        bool findFlag = false;

    public:
        astarSearch(){};
        ~astarSearch(){};
        void readingMap(const std::string& mapPath);
        void initialMap(std::string mapPath);
        bool isValid(int coord_row, int coord_col);
        bool isObstacle(int coord_row, int coord_col); 
        std::vector<Eigen::Vector2d> recoverPath();
        void getSucc(nodePtr currentPtr, std::vector<nodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets);
        double getHeu(nodePtr p1, nodePtr p2);
        void resetNodes(nodePtr ptr);
        void resetUsedNodes();
        bool run( Eigen::Vector2d start,  Eigen::Vector2d goal,  std::string& mapPath);
        
};


void astarSearch::readingMap(const std::string& mapPath){
    std::ifstream file(mapPath);
    
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<int> row;
            for (char c : line) {
                // 将字符 '0' 转换为整数 0，将字符 '1' 转换为整数 1
                if (c != ' ') {
                    row.push_back(c - '0');
                }
            }
            astarMap.push_back(row);
        }
        row_SIZE = astarMap.size();
        col_SIZE = astarMap[1].size();
        
        file.close();
    } else {
        std::cerr << "Unable to open file: " << mapPath << std::endl;
    }

}

inline bool astarSearch::isValid(int coord_row, int coord_col){
    return (coord_row >0 && coord_row < row_SIZE && coord_col > 0 && coord_col < col_SIZE);
}

inline bool astarSearch::isObstacle(int coord_row, int coord_col){
    return astarMap[coord_row -1 ][coord_col - 1] == 1;
}

std::vector<Eigen::Vector2d> astarSearch::recoverPath(){
    std::vector<Eigen::Vector2d> path;
    std::vector<nodePtr> gridPath;

    nodePtr currentPtr = goalPtr;
    while (currentPtr != NULL){
        gridPath.push_back(currentPtr);
        currentPtr = currentPtr -> parentNodePtr;
    }
    for (auto ptr: gridPath){
        path.push_back(Eigen::Vector2d(ptr->coord_row, ptr->coord_col));
    }
    std::reverse(path.begin(),path.end());
    return path;
 }
void astarSearch::getSucc(nodePtr currentPtr, std::vector<nodePtr> & neighborPtrSets, std::vector<double> & edgeCostSets){
    neighborPtrSets.clear();
    edgeCostSets.clear();
    double current_x = currentPtr->coord_row;
    double current_y = currentPtr->coord_col;

    nodePtr neighborPtr;
    for (int i = current_x - 1; i <= current_x + 1; i++){
        for (int j = current_y - 1; j <= current_y + 1; j++){
            if (isValid(i,j)){


            if (!isObstacle(i, j)){
                neighborPtr = nodeMapPtr[i][j];
            }
            else continue;

            }

            if (neighborPtr != currentPtr){
                neighborPtrSets.push_back(neighborPtr);
                edgeCostSets.push_back(sqrt(abs(current_x - i) + abs(current_y - j)));
                
            }
        }
    }
}
double astarSearch::getHeu(nodePtr p1, nodePtr p2){
    return sqrt(pow((p1->coord_row - p2->coord_row), 2) + pow((p1->coord_col - p2->coord_col), 2));
}
void astarSearch::resetNodes(nodePtr ptr){
    ptr->id = 0;
    ptr->parentNodePtr = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}
void astarSearch::initialMap(std::string mapPath){
    readingMap(mapPath);
    // nodePtr nodeMapPtr_[row_SIZE][col_SIZE];
    // nodeMapPtr = new nodePtr * [row_SIZE];
    for (auto i = 0; i < row_SIZE; i++){
        // nodeMapPtr[i] = new nodePtr  [col_SIZE];
        for (auto j = 0; j < col_SIZE; j++){
            nodeMapPtr[i][j] = new node(i, j);
        }
    }

    for (auto i = 0; i < row_SIZE; i++){
        for (auto j = 0; j < col_SIZE; j++){
            resetNodes(nodeMapPtr[i][j]);
        }
    }
}

void astarSearch::resetUsedNodes(){
    for (auto i = 0; i < row_SIZE; i++){
        for (auto j = 0; j < col_SIZE; j++){
            resetNodes(nodeMapPtr[i][j]);
        }
    }
}

bool astarSearch::run(Eigen::Vector2d start, Eigen::Vector2d goal, std::string& mapPath){
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // 1, <-----------------initialize map and nodes----------------->
    initialMap(mapPath);
    
    nodePtr startPtr = new node(int(start.x()), int(start.y()));
    nodePtr endPtr = new node(int(goal.x()), int(goal.y()));

    openSet.clear();
    nodePtr currentPtr  = NULL;
    nodePtr neighborPtr = NULL;

    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr -> id = 1; 
    
    openSet.insert(std::make_pair(startPtr->fScore, startPtr));

    std::vector<nodePtr> closeSet;
    std::vector<nodePtr> neighborPtrSets;
    std::vector<double> edgeCostSets;


    // 2, <-----------------initialize map and nodes----------------->
    while ( !openSet.empty() ){

        currentPtr = openSet.begin() -> second;
        currentPtr -> id = -1;
        openSet.erase( openSet.begin() ); 
        closeSet.push_back( currentPtr );

        // check whether current node is the goal

        if( currentPtr->coord_row == endPtr->coord_row && currentPtr->coord_col == endPtr->coord_col ){
            goalPtr = currentPtr;
            findFlag = true;
            break;
        }

        // expand 
        getSucc(currentPtr, neighborPtrSets, edgeCostSets);

        // 
        for ( auto i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            // never been visited
            if (neighborPtr -> id == 0){
                neighborPtr -> gScore = currentPtr -> gScore + getHeu(neighborPtr, currentPtr);
                neighborPtr -> fScore = getHeu(neighborPtr, endPtr);
                neighborPtr -> id = 1;
                neighborPtr -> parentNodePtr = currentPtr;
                openSet.insert( std::make_pair(neighborPtr -> gScore + neighborPtr -> fScore, neighborPtr) );

                continue;
            }
            // visited before, check whether need to update
            else if (neighborPtr -> id == 1){
                if(neighborPtr -> gScore > currentPtr -> gScore + getHeu(neighborPtr, currentPtr))
                {
                    neighborPtr -> gScore = currentPtr -> gScore + getHeu(currentPtr, neighborPtr);
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr -> parentNodePtr = currentPtr;
                    openSet.erase(neighborPtr -> fScore);
                    openSet.insert( std::make_pair(neighborPtr -> fScore, neighborPtr));
                    
                }
                continue;
            }
            else{


                continue;
            }
        }
    }
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    if (findFlag)
    {
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "path found ! time used : " << time_used.count() << std::endl;
    }
    
    return findFlag;

}


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

        // 遍历 vector，并将每个 Eigen::Vector2d 的 x 和 y 写入一行
        for (const auto& vec : grid_path) {
            outFile << vec.x() << " " << vec.y() << std::endl;
        }

        // 关闭文件
        outFile.close();
        std::cout << "planning successfully , path length :" << std::endl;
    }
    else{

    std::cout <<"Error: Unable to find the path" << std::endl;

    }

    return 0;
}