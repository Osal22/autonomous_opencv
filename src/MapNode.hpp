#ifndef MAP_NODE_HPP__
#define MAP_NODE_HPP__
#pragma once
#include "utils.h"

class MapNode {
public:
    int x = -1;
    int y = -1;
    int h = 0;
    int g = 0;
    int type = NODE_TYPE_ZERO;
    int flag = NODE_FLAG_UNDEFINED;
    MapNode *parent = 0;

    MapNode() { }

    MapNode(int x, int y, int type = NODE_TYPE_ZERO, int flag = NODE_FLAG_UNDEFINED, MapNode *parent = 0) {
        this->x = x;
        this->y = y;
        this->type = type;
        this->flag = flag;
        this->parent = parent;
    }

    int f() {
        return g + h;
    }
};

#endif