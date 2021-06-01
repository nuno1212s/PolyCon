#include <iostream>
#include <vector>
#include "polygon_construction.h"
#include "dcel/dcel.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::vector<std::vector<int>> points;

    points.push_back({4, 4});
    points.push_back({7, 7});
    points.push_back({9, 7});
    points.push_back({11, 8});
    points.push_back({13, 6});
    points.push_back({14, 4});
    points.push_back({12, 3});
    points.push_back({10, 2});
    points.push_back({7, 1});

    sortPointsInCCWOrder_2D(points);

    for (const auto &ponto : points) {
        std::cout << ponto[0] << " " << ponto[1] << std::endl;
    }

    DCEL dcel;

    dcel.initialize(points);

    std::cout << "Hello " << std::endl;

    return 0;
}