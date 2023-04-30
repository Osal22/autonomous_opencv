#include "Map.hpp"

Map::Map()
{

}

void Map::hello()
{
}

MapNode *Map::mapAt(int x, int y)
{
    if (x < 0 || y < 0 || x >= mapSize.width || y >= mapSize.height)return 0;
    return &mapData[y * mapSize.width + x];
}

std::vector<MapNode *> Map::neighbors(MapNode *node) {
    std::vector<MapNode *> available;
    MapNode *_node;

    // L
    if ((_node = mapAt(node->x - 1, node->y)) != 0)available.push_back(_node);
    // T
    if ((_node = mapAt(node->x, node->y - 1)) != 0)available.push_back(_node);
    // R
    if ((_node = mapAt(node->x + 1, node->y)) != 0)available.push_back(_node);
    // B
    if ((_node = mapAt(node->x, node->y + 1)) != 0)available.push_back(_node);

    if (ALLOW_VERTEX_PASSTHROUGH) {
        // LT
        if ((_node = mapAt(node->x - 1, node->y - 1)) != 0)available.push_back(_node);
        // RT
        if ((_node = mapAt(node->x + 1, node->y - 1)) != 0)available.push_back(_node);
        // RB
        if ((_node = mapAt(node->x + 1, node->y + 1)) != 0)available.push_back(_node);
        // LB
        if ((_node = mapAt(node->x - 1, node->y + 1)) != 0)available.push_back(_node);
    }

    return available;
}

int Map::computeH(MapNode *node1, MapNode *node2) {
    // return abs(node1->x - node2->x) + abs(node1->y - node2->y);
    if (ALLOW_VERTEX_PASSTHROUGH) {
        return diagonal_distance(node1, node2)*G_SKEW;
    } else {
        return manhattan_distance(node1, node2)*G_DIRECT;
    }
}

int Map::computeG(MapNode *node1, MapNode *node2) {
    int dX = abs(node1->x - node2->x);
    int dY = abs(node1->y - node2->y);
    if (dX > dY) {
        return 14 * dY + 10 * (dX - dY);
    } else {
        return 14 * dX + 10 * (dY - dX);
    }
}

std::vector<MapNode *> Map::find() {
    std::vector<MapNode *> path;
    path.clear();
    std::cout << "Finding started!" << std::endl;
    int iteration = 0;
    MapNode *node;
    MapNode *reversedPtr = 0;

    while (openList.size() > 0) {
        // std::cout<<"openlist "<<openList.size()<<std::endl;;
        node = openList.at(0);

        for (int i = 0, max = openList.size(); i < max; i++) {
            if (openList[i]->f() <= node->f() && openList[i]->h < node->h) {
                node = openList[i];
            }
        }
        // std::cout<<"in openlist 2 \n";
        openList.erase(remove(openList.begin(), openList.end(), node), openList.end());
        node->flag = NODE_FLAG_CLOSED;
        // std::cout << iteration++ << std::endl;
        // std::cout << "   Current node " << node->x << ", " << node->y << " ..." << std::endl;
        if (node->parent != 0)
            // std::cout << "       ... parent " << node->parent->x << ", " << node->parent->y << std::endl;
        if (node == targetNode) {
            std::cout << "Reached the target node." << std::endl;
            reversedPtr = node;
            draw=true;
            break;
        }
        std::vector<MapNode *> neighborNodes = neighbors(node);
        // cout << "       ... has " << neighborNodes.size() << " neighbors" << endl;
        for (int i = 0; i < neighborNodes.size(); i++) {
            MapNode *_node = neighborNodes[i];
            if (_node->flag == NODE_FLAG_CLOSED || _node->type == NODE_TYPE_OBSTACLE) {
                continue;
            }
            int g = node->g + computeG(_node, node);
            if (_node->flag == NODE_FLAG_UNDEFINED || g < _node->g) {
                _node->g = g;
                _node->h = computeH(_node, targetNode);
                _node->parent = node;
                if (_node->flag != NODE_FLAG_OPEN) {
                    _node->flag = NODE_FLAG_OPEN;
                    openList.push_back(_node);
                }
            }
        }
        drawOpenList();
        if (openList.size() <= 0) break;

    }
    if (reversedPtr == 0) {

        std::cout << "Target node is unreachable." <<std::endl;
        draw=false;
    } else {
        MapNode *_node = reversedPtr;
        while (_node->parent != 0) {
            path.push_back(_node);
            _node = _node->parent;
        }
        reverse(path.begin(), path.end());
    }
    return path;
}

void Map::drawOpenList() {
    for (int i = 0; i < openList.size(); i++) {
        MapNode *node = openList[i];
        if (node == startNode || node == targetNode)continue;
        map_1.at<cv::Vec3b>(node->y, node->x) = cv::Vec3b(210, 210, 210);
    }
}
void Map::drawPath(cv::Mat &map, std::vector<MapNode *> path)
    {

    }

inline int Map::manhattan_distance(MapNode* node1, MapNode* node2){
    return abs(node2->x - node1->x) + abs(node2->y - node1->y);
}

 inline int Map::diagonal_distance(MapNode* node1, MapNode* node2)
 {
    return std::max(abs(node2->x - node1->x),abs(node2->y - node1->y));
 }

 void Map::make_nodes()
 {
    for (int y = 0; y < map_1.rows; y++) {
        for (int x = 0; x < map_1.cols; x++) {
            if (map_1.at<cv::Vec3b>(y, x) == cv::Vec3b(255, 255, 255)) 
            {
                 mapData[y *  mapSize.width + x] = MapNode(x, y, NODE_TYPE_ZERO);
            } 
            else if (map_1.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 0)) 
            {
                 mapData[y *  mapSize.width + x] = MapNode(x, y, NODE_TYPE_OBSTACLE);
            } 
            else if (map_1.at<cv::Vec3b>(y, x) == cv::Vec3b(255, 0, 0)) 
            {
                MapNode node(x, y, NODE_TYPE_START);
                 mapData[y *  mapSize.width + x] = node;
                startNode = & mapData[y *  mapSize.width + x];

            } 
            else if (map_1.at<cv::Vec3b>(y, x) == cv::Vec3b(0, 0, 255)) 
            {
                MapNode node(x, y, NODE_TYPE_END);
                 mapData[y *  mapSize.width + x] = node;
                targetNode = & mapData[y *  mapSize.width + x];

            } 
            else 
            {
                map_1.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
                mapData[y *  mapSize.width + x] = MapNode(x, y, NODE_TYPE_OBSTACLE);
            }
        }

    }
 }

    std::vector<MapNode *> Map::get_path()
    {
        return find();
    }
