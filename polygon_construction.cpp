#include <algorithm>
#include "polygon_construction.h"
#include <iostream>
#include <random>
#include <fstream>
#include <math.h>

#define X(point) (point[0])
#define Y(point) (point[1])

static std::vector<int> center;

template<typename T>
std::vector<T> calculateCentroid(const std::vector<point<T>>

                                 &points) {

    if (points.

            empty()

            )
        return

                std::vector<T>();

    int dimen = points[0].size();

    std::vector<int> sumPoints = std::vector<T>(dimen);

    for (
        const auto &point
            : points) {

        for (
                int i = 0;
                i < dimen;
                i++) {
            sumPoints[i] += point[i];
        }

    }

    for (
            int i = 0;
            i < dimen;
            i++) {
        sumPoints[i] /= points.

                size();

    }

    std::cout << "Centroid: " << X(sumPoints) << " " << Y(sumPoints) <<
              std::endl;

    return
            sumPoints;
}

template<typename T>
inline bool arePointsEqual(const point<T> &p1, const point<T> &p2) {
    return (X(p1) == X(p2) && Y(p1) == Y(p2));
}

template<typename T>
float triangleArea(const point<T> &p1, const point<T> &p2, const point<T> &p3) {

    float part1 = (X(p1) * (Y(p2) - Y(p3))),
            part2 = (X(p2) * (Y(p3) - Y(p1))),
            part3 = (X(p3) * (Y(p1) - Y(p2)));

    return std::abs((part1 + part2 + part3) / 2.0);
}

template<typename T>
bool isCollinear(const point<T> &point1, const point<T> &point2, const point<T> &point3) {

    T area = X(point1) * (Y(point2) - Y(point3)) +
             X(point2) * (Y(point3) - Y(point1)) +
             X(point3) * (Y(point1) - Y(point2));

    return area == 0;
}

template<typename T>
int orientation(const point<T> &p1, const point<T> &p2, const point<T> &p3) {
    int val = (Y(p2) - Y(p1)) * (X(p3) - X(p2)) -
              (X(p2) - X(p1)) * (Y(p3) - Y(p2));

    if (val == 0) return 0; // colinear

    return (val > 0) ? 1 : 2;// clock or counterclock wise
}

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
template<typename T>
bool onSegment(const point<T> &p, const point<T> &q, const point<T> &r) {
    if (X(q) <= std::max(X(p), X(r)) && X(q) >= std::min(X(p), X(r)) &&
        Y(q) <= std::max(Y(p), Y(r)) && Y(q) >= std::min(Y(p), Y(r)))
        return true;

    return false;
}

template<typename T>
bool isContainedInTriangle(const point<T> &point, const std::vector<::point<T>>
&trianglePoints) {

    if (trianglePoints.
            size() < 3 || trianglePoints.size() > 3) {

        std::cerr << "Not a triangle" <<
                  std::endl;

        exit(EXIT_FAILURE);
    }

    float area = triangleArea(trianglePoints[0], trianglePoints[1], trianglePoints[2]);

    float area1 = triangleArea(point, trianglePoints[1], trianglePoints[2]),
            area2 = triangleArea(trianglePoints[0], point, trianglePoints[2]),
            area3 = triangleArea(trianglePoints[0], trianglePoints[1], point);

    return area == (area1 + area2 + area3);
}

template<typename T>
bool doSegmentsOverlap(const std::tuple<point<T>, point<T>>

                       &seg1, const std::tuple<point<T>, point<T>> &seg2,
                       bool comparePoints
) {

    point<T> p1, p2, q1, q2;

    std::tie(p1, p2
    ) =
            seg1;
    std::tie(q1, q2
    ) =
            seg2;

    float p0_x = X(p1), p0_y = Y(p1),
            p1_x = X(p2), p1_y = Y(p2),
            p2_x = X(q1), p2_y = Y(q1),
            p3_x = X(q2), p3_y = Y(q2);

    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;
    s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;
    s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (!comparePoints) {
        if (s > 0 && s < 1 && t > 0 && t < 1) {
            return true;
        }
    } else {
        if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
            return true;
        }
    }

    return false; // No collision
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

template<typename T>
bool isCollinearWithAny(Face *face, const point<T> &p) {

    HalfEdge *curr = face->getArbitraryHalfEdge();

    do {

        if (isCollinear(p, curr->getOriginVertex()->point(), curr->getTargetVertex()->point())) {
            return true;
        }

        curr = curr->getNext();
    } while (curr != face->getArbitraryHalfEdge());

    return false;
}

static void traverseFace(Face *f) {

    auto edge = f->getArbitraryHalfEdge();

    auto start = edge->getNext();

    std::cout << "Traversing face " << start << std::endl;

    std::cout << "Start: " << start << " " << edge << std::endl;

    while (start != edge) {
        auto v = start->getTargetVertex();
        std::cout << v->point()[0] << " " << v->point()[1] << std::endl;

        start = start->getNext();
    }

    auto v = start->getTargetVertex();
    std::cout << v->point()[0] << " " << v->point()[1] << std::endl;
}

bool isContainedInPolygon(DCEL *dcel, Face *outerFace, const point<int> &point) {

    std::vector<::point<int>>
            points;

    points.reserve(3);

    for (const auto &face : dcel->getFaces()) {

        if (face.get() == outerFace) continue;

        auto edge = face->getArbitraryHalfEdge();

        do {

            points.push_back(edge->getTargetVertex()->point());

            edge = edge->getNext();

        } while (edge != face->getArbitraryHalfEdge());

        if (isContainedInTriangle(point, points)) {
            return true;
        }

        points.clear();
    }

    return false;
}

bool overlapsWithPolygon(Face *outerFace, const point<int> &p1, const point<int> &v, const point<int> &q) {

    auto segment_p1_v = std::make_tuple(p1, v),
            segment_p1_q = std::make_tuple(p1, q);

    auto edge = outerFace->getArbitraryHalfEdge(), first = edge;

    //std::cout << std::endl << "Checking if overlaps " <<
    //          X(p1) << "," << Y(p1) << ":" << X(q) << "," << Y(q) << std::endl
    //          << X(p1) << "," << Y(p1) << ":" << X(v) << "," << Y(v) << " " << std::endl;

    do {
        auto origin = edge->getTargetVertex(),
                target = edge->getTwin()->getTargetVertex();

        auto segment = std::make_tuple(origin->point(), target->point());

        //std::cout << "Checking if segment " << X(origin->point()) << " " << Y(origin->point())
        //          << ":" << X(target->point()) << " " << Y(target->point()) << std::endl;

        bool testPoints = true;

        if ((arePointsEqual(origin->point(), q) || arePointsEqual(target->point(), v)) ||
            (arePointsEqual(origin->point(), v) || arePointsEqual(target->point(), q))) {
            testPoints = false;
        }

//        std::cout << "Testing vs " << X(origin->point()) << " " << Y(origin->point()) << ", " <<
//                  X(target->point()) << " " << Y(target->point()) << " " << testPoints << std::endl;

        if (doSegmentsOverlap(segment, segment_p1_q, testPoints) ||
            doSegmentsOverlap(segment, segment_p1_v, testPoints)) {
            return true;
        }

        edge = edge->getNext();

    } while (edge != first);

    return false;
}

void testPoints(int M);

std::unique_ptr<DCEL> generateIntNPointsWithinM(int N, int M) {

    /*testPoints(M);

    return std::make_unique<DCEL>();*/

    std::random_device randomEngine;
    std::mt19937 generator(randomEngine());

    std::uniform_int_distribution<int> distribution(0, M);

    auto dcel = std::make_unique<DCEL>();

    std::vector<point<int>>
            points;

    if (N < 3) {
        std::cerr << "Cannot have a polygon with less than 3 vertices" << std::endl;

        return std::move(dcel);
    }

    point<int> p1{distribution(generator), distribution(generator)},
            p2{distribution(generator), distribution(generator)};

    while (arePointsEqual(p1, p2)) {
        p1 = point<int>{distribution(generator), distribution(generator)};
        p2 = point<int>{distribution(generator), distribution(generator)};;
    }

    points.push_back(p1);
    points.push_back(p2);

    point<int> p3{distribution(generator), distribution(generator)};

    while (isCollinear(points[0], points[1], p3)) {
        p3 = point<int>{distribution(generator), distribution(generator)};
    }

    points.push_back(p3);

    //sortPointsInCCWOrder_2D(points);

    Face *outer = dcel->initialize(points);

    dcel->print();

    int verts = 3;

    forv:
    for (; verts < N;) {

        point<int> nP{distribution(generator), distribution(generator)};

        if (isContainedInPolygon(dcel.get(), outer, nP)) {
            continue;
        }

        std::uniform_int_distribution<int> dist(0, outer->getEdgeCount());

        int face = dist(generator), curr = 0;

        auto start = outer->getArbitraryHalfEdge();

        do {
            start = start->getNext();

            curr++;
        } while (curr < face);

        auto target = start->getTargetVertex(),
                origin = start->getOriginVertex();

        if (X(nP) == X(target->point()) || X(nP) == X(origin->point())
            || Y(nP) == Y(target->point()) || Y(nP) == Y(origin->point())) {
            continue;
        }

        bool overlaps = false;

        if (overlapsWithPolygon(outer, nP, origin->point(),
                                target->point())) {
            overlaps = true;
            continue;
        }

        if (overlaps) continue;

        if (isCollinearWithAny(outer, nP)) {
            continue;
        }

        auto added = dcel->addVertex(nP, start);

        outer = dcel->addEdge(std::get<1>(added), origin);

        verts++;

        std::cout << "Added point " << X(nP) << "," << Y(nP) << " to the DCEL to the edge " <<
                  X(origin->point()) << ", " << Y(origin->point())
                  << " " << X(target->point()) << ", " << Y(target->point()) <<
                  ". Current state:" << verts << "/" << N << std::endl;

    }

    dcel->print();

    std::ofstream file("result.json");

    file << *dcel->serialize();

    file.close();
}

void testPoints(int M) {

    std::random_device randomEngine;
    std::mt19937 generator(randomEngine());

    std::uniform_int_distribution<int> distribution(0, M);

    std::vector<point<int>>
            points;

    auto dcel = std::make_unique<DCEL>();

    points.push_back({7, 5});
    points.push_back({9, 8});
    points.push_back({9, 10});

    Face *nF = dcel->initialize(points);

    HalfEdge *edge = nF->getArbitraryHalfEdge();

    Vertex *v1 = edge->getTargetVertex(), *v2 = edge->getTwin()->getTargetVertex();

    HalfEdge *prevV2 = edge;

    while (prevV2->getTargetVertex() != v2) {
        prevV2 = prevV2->getNext();
    }

    std::cout << "Chose edge " << X(v1->point())
              << " " << Y(v1->point()) << ","
              << X(v2->point())
              << " " << Y(v2->point()) << std::endl;

    point<int> nP{1, 4};

    std::cout << "Point " << X(nP) << "," << Y(nP) << " overlaps? "
              << overlapsWithPolygon(nF, nP, edge->getOriginVertex()->point(),
                                     edge->getTargetVertex()->point())
              << std::endl;

    auto added = dcel->addVertex(nP, edge);

    nF = dcel->addEdge(std::get<1>(added), v2);

    traverseFace(nF);
}