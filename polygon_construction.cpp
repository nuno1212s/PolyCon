#include <algorithm>
#include "polygon_construction.h"
#include <iostream>

#define X(point) (point[0])
#define Y(point) (point[1])

static std::vector<int> center;

std::vector<int> calculateCentroid(const std::vector<std::vector<int>> &points) {

    if (points.empty()) return std::vector<int>();

    int dimen = points[0].size();

    std::vector<int> sumPoints = std::vector<int>(dimen);

    for (const auto &point : points) {

        for (int i = 0; i < dimen; i++) {
            sumPoints[i] += point[i];
        }

    }

    for (int i = 0; i < dimen; i++) {
        sumPoints[i] /= points.size();
    }

    std::cout << "Centroid: " << X(sumPoints) << " " << Y(sumPoints) <<std::endl;

    return sumPoints;
}

bool comparePoint2D(std::vector<int> i1, std::vector<int> i2) {

    if (X(i1) - X(center) >= 0 && X(i2) - X(center) < 0) {
        return true;
    } else if (X(i1) - X(center) < 0 && X(i2) - X(center) >= 0) {
        return false;
    } else if (X(i1) - X(center) == 0 && X(i2) - X(center) == 0) {

        if (Y(i1) - Y(center) >= 0 || Y(i2) - Y(center) >= 0) {
            return Y(i1) > Y(i2);
        }

        return Y(i2) > Y(i1);
    } else {

        int det = (X(i1) - X(center)) * (Y(i2) - Y(center)) - (X(i2) - X(center)) * (Y(i1) - Y(center));

        if (det < 0) return true;
        else if (det > 0) return false;

        int dist1 = (X(i1) - X(center)) * (X(i1) - X(center)) + (Y(i1) - Y(center)) * (Y(i1) - Y(center)),
                dist2 = (X(i2) - X(center)) * (X(i2) - X(center)) + (Y(i2) - Y(center)) * (Y(i2) - Y(center));

        return dist1 > dist2;
    }

}

bool notCompare2D(std::vector<int> i1, std::vector<int> i2) {
    return !comparePoint2D(std::move(i1), std::move(i2));
}

void sortPointsInCCWOrder_2D(std::vector<std::vector<int>> &points) {

    auto centroid = calculateCentroid(points);

    center = centroid;

    std::sort(points.begin(), points.end(), notCompare2D);
}
