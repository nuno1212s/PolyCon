#ifndef TRABALHO2_POLYGON_CONSTRUCTION_H
#define TRABALHO2_POLYGON_CONSTRUCTION_H

#include <vector>

#include "dcel/dcel.h"

template<typename T>
using point = std::vector<T>;

template<typename T>
void sortPointsInCCWOrder_2D(std::vector<point<T>> &points);

std::unique_ptr<std::vector<point<int>>> generateIntNPointsWithinM(int N, int M);

#endif //TRABALHO2_POLYGON_CONSTRUCTION_H
