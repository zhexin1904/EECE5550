// HW4 Astar of EECE5550
// work flow of astar 
// zhexin xu (xu.zhex@northeastern.edu)
#include "/home/jason/EECE5550/hw/HW4/astar/include/astar.h"

// using namespace std;

void astarSearch::readingMap(const std::string& mapPath){
    std::ifstream file(mapPath);
    
    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
            std::vector<int> row;
            for (char c : line) {
                row.push_back(c - '0');
            }
            astarMap.push_back(row);
        }
        row_SIZE = astarMap.size();
        col_SIZE = astarMap.at(1).size();
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
        path.push_back(ptr->coord);
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
    for (auto i = current_x - 1; i <= current_x + 1; i++){
        for (auto j = current_y - 1; j < current_y + 1; j++){
            if (!isObstacle(current_x, current_y)){
                neighborPtr = new node(current_x, current_y);
            }
            else continue;
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
    nodeMapPtr = new nodePtr * [row_SIZE];
    for (auto i = 0; i < row_SIZE; i++){
        nodeMapPtr[i] = new nodePtr  [col_SIZE];
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
                neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                neighborPtr -> fScore = getHeu(neighborPtr, endPtr);
                neighborPtr -> id = 1;
                neighborPtr -> parentNodePtr = currentPtr;
                openSet.insert( std::make_pair(neighborPtr -> gScore + neighborPtr -> fScore, neighborPtr) );

                continue;
            }
            // visited before, check whether need to update
            else if (neighborPtr -> id == 1){
                if ( currentPtr -> gScore + edgeCostSets[i] < neighborPtr -> gScore){
                    double key = neighborPtr -> gScore + neighborPtr -> fScore;
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> parentNodePtr = currentPtr;
                    openSet.erase( key );
                    openSet.insert( std::make_pair(neighborPtr -> gScore + neighborPtr -> fScore, neighborPtr) );
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
