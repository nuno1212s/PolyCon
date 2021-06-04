#include <algorithm>
#include "polygon_construction.h"
#include <iostream>
#include <random>
#include <math.h>

#define X(point) (point[0])
#define Y(point) (point[1])

static std::vector<int> center;

template<typename T>
std::vector<T> calculateCentroid(const std::vector<point<T>> &points) {

    if (points.empty()) return std::vector<T>();

    int dimen = points[0].size();

    std::vector<int> sumPoints = std::vector<T>(dimen);

    for (const auto &point : points) {

        for (int i = 0; i < dimen; i++) {
            sumPoints[i] += point[i];
        }

    }

    for (int i = 0; i < dimen; i++) {
        sumPoints[i] /= points.size();
    }

    std::cout << "Centroid: " << X(sumPoints) << " " << Y(sumPoints) << std::endl;

    return sumPoints;
}

template<typename T>
bool comparePoint2D(std::vector<T> i1, std::vector<T> i2) {

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

template<typename T>
bool notCompare2D(point<T> i1, point<T> i2) {
    return !comparePoint2D(std::move(i1), std::move(i2));
}

template<typename T>
void sortPointsInCCWOrder_2D(std::vector<point<T>> &points) {

    auto centroid = calculateCentroid(points);

    center = centroid;

    std::sort(points.begin(), points.end(), notCompare2D);
}

std::unique_ptr<DCEL> generateNPointsWithinM(int N, int M) {

    std::random_device randomEngine;
    std::mt19937 generator(randomEngine());

    auto points = std::make_unique<std::vector<point<int>>>(N);

    int leftXMin = 0, leftXMax = std::floor(M / 4.0),
            YMin = std::floor(M / 4.0), YMax = std::floor((M / 4.0) * 3),
            rightXMin = std::ceil((M / 4.0) * 3), rightXMax = M,
            topYMin = std::ceil(M / 2.0), topYMax = M,
            bottomYMin = std::floor(M / 2.0), bottomYMax = 0,
            XMin = std::floor(M / 4.0), XMax = std::floor((M / 4.0) * 3);

    int leftVertices = N / 4 + (N % 4 > 0 ? 1 : 0), rightVertices = N / 4 + (N % 4 > 1 ? 1 : 0),
            topVertices = N / 4 + (N % 4 > 2 ? 1 : 0), bottomVertices = N / 4;

    {
        //Generate the N/4 vertices on the left
        //The left polygons will be generated with an always increasing Y.
        int previous = YMin;

        int yVal = leftXMin;

        int yIncrement = std::floor((YMax - YMin) / leftVertices);

        for (int i = 0; i < leftVertices; i++) {

            int xVal = -1;

            if (i % 2 == 0) {
                std::uniform_int_distribution<int> distribution(previous, leftXMax);

                xVal = distribution(generator);
            } else {
                std::uniform_int_distribution<int> distribution(leftXMin, previous);

                xVal = distribution(generator);
            }

            points->emplace_back(xVal, yVal);

            yVal += yIncrement;
        }
    }

    {
        //Generate the N/4 vertices on the right
        int previous = rightXMin;

        int yVal = YMin;

        int yIncrement = std::floor((YMax - YMin) / rightVertices);

        for (int i = 0; i < rightVertices; i++) {
            int xVal = -1;

            if (i % 2 == 0) {
                std::uniform_int_distribution<int> distribution(previous, rightXMax);

                xVal = distribution(generator);
            } else {
                std::uniform_int_distribution<int> distribution(rightXMin, previous);

                xVal = distribution(generator);
            }

            points->emplace_back(xVal, yVal);

            yVal += yIncrement;
        }
    }

    {
        //Generate the N / 4 vertices on top
        int previous = topYMin;

        int xVal = XMin;

        int xIncrement = std::floor((XMax - XMin) / topVertices);

        for (int i = 0; i < topVertices; i++) {
            int yVal = -1;

            if (i % 2 == 0) {
                std::uniform_int_distribution<int> distribution(previous, topYMax);

                yVal = distribution(generator);
            } else {
                std::uniform_int_distribution<int> distribution(topYMin, previous);

                yVal = distribution(generator);
            }

            points->emplace_back(xVal, yVal);

            xVal += xIncrement;
        }
    }

    {
        //Generate the N / 4 vertices on bottom
        int previous = bottomYMin;

        int xVal = XMin;

        int xIncrement = std::floor((XMax - XMin) / bottomVertices);

        for (int i = 0; i < bottomVertices; i++) {
            int yVal = -1;

            if (i % 2 == 0) {
                std::uniform_int_distribution<int> distribution(previous, bottomYMax);

                yVal = distribution(generator);
            } else {
                std::uniform_int_distribution<int> distribution(bottomYMin, previous);

                yVal = distribution(generator);
            }
            points->emplace_back(xVal, yVal);

            xVal += xIncrement;
        }
    }

    return std::move(points);
}
