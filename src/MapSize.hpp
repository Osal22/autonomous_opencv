#ifndef MAP_SIZE_HPP__
#define MAP_SIZE_HPP__
#pragma once
#include "utils.h"


class MapSize {
public:
    unsigned long width = 0;
    unsigned long height = 0;
    unsigned long size = 0;

    MapSize() { }

    MapSize(unsigned long width, unsigned long height) {
        this->width = width;
        this->height = height;
        this->size = width * height;
    }
};

#endif //MAP_SIZE