#ifndef MAP_HPP
#define MAP_HPP
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "utils.h"
#include <vector>
#include "MapNode.hpp"
#include "MapSize.hpp"



class Map
{
private:
    

public:
    Map();
    ~Map();

    void hello();
    cv::Mat map_1;
    bool draw=true;
    MapSize mapSize;
    std::vector<MapNode> mapData;
    std::vector<MapNode *> openList;

    MapNode *startNode;
    MapNode *targetNode;

    MapNode *mapAt(int x, int y);

    std::vector<MapNode *> neighbors(MapNode *node);

    int computeH(MapNode *node1, MapNode *node2);

    int computeG(MapNode *node1, MapNode *node2);

    std::vector<MapNode *> find();

    void drawPath(cv::Mat &map, std::vector<MapNode *> path);

    void drawOpenList();

    inline int manhattan_distance(MapNode* node1, MapNode* node2);

    inline int diagonal_distance(MapNode* node1, MapNode* node2);
    void make_nodes();
    std::vector<MapNode *> get_path();
    std::vector<MapNode *> path_prev;
    std::vector<cv::Point>obstacle_vec;
    std::vector<cv::Point> get_obstacle_vec();
   
};
#endif //MAP_HPP



