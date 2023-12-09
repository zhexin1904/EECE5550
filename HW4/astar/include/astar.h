#ifndef _ASTAR_H__
#define _ASTAR_H__

#include <iostream>
#include <Eigen/Eigen>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
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
        int row_SIZE, col_SIZE;
        nodePtr goalPtr;

        std::multimap<double, nodePtr> openSet;

        nodePtr** nodeMapPtr;

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


















#endif