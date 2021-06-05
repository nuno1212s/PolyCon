#ifndef TRABALHO2_DCEL_H
#define TRABALHO2_DCEL_H

#include <utility>
#include <vector>
#include <memory>
#include "nlohmann/json.hpp"

#define X(point) (point[0])
#define Y(point) (point[1])

class Vertex;

class Face;

class HalfEdge {

private:
    Vertex *targetVertex, *originVertex;

    Face *incidentFace;

    HalfEdge *twin, *next, *previous;

public:
    Vertex *getOriginVertex() const {
        return originVertex;
    }

    Vertex *getTargetVertex() const {
        return targetVertex;
    }

    Face *getIncidentFace() const {
        return incidentFace;
    }

    HalfEdge *getTwin() const {
        return twin;
    }

    HalfEdge *getNext() const {
        return next;
    }

    HalfEdge *getPrevious() const {
        return previous;
    }

    void setOriginVertex(Vertex *origin) {
        HalfEdge::originVertex = origin;
    }

    void setTargetVertex(Vertex *targetVertex) {
        HalfEdge::targetVertex = targetVertex;
    }

    void setIncidentFace(Face *incidentFace) {
        HalfEdge::incidentFace = incidentFace;
    }

    void setTwin(HalfEdge *twin) {
        HalfEdge::twin = twin;
    }

    void setNext(HalfEdge *next) {
        HalfEdge::next = next;
    }

    void setPrevious(HalfEdge *previous) {
        HalfEdge::previous = previous;
    }

};

class Vertex {

private:
    HalfEdge *halfEdge;

    std::vector<int> coordinates;

public:
    Vertex(std::vector<int> coordinates) : coordinates(std::move(coordinates)) {
    }

public:
    HalfEdge *getHalfEdge() const {
        return halfEdge;
    }

    const std::vector<int> &point() const {
        return this->coordinates;
    }

    void setHalfEdge(HalfEdge *halfEdge) {
        Vertex::halfEdge = halfEdge;
    }

};

class Face {

private:
    HalfEdge *arbitraryHalfEdge;
public:

    Face(HalfEdge *edge) : arbitraryHalfEdge(edge) {}

    HalfEdge *getArbitraryHalfEdge() const {
        return arbitraryHalfEdge;
    }

    int getEdgeCount() const {

        int count = 0;

        HalfEdge *start = getArbitraryHalfEdge();

        do {
            count++;

            start = start->getNext();
        } while (start != getArbitraryHalfEdge());

        return count;
    }
};

/**
 * Doubly connected edge list
 */
class DCEL {

private:
    std::vector<std::unique_ptr<HalfEdge>> halfEdges;

    std::vector<std::unique_ptr<Vertex>> vertexes;

    std::vector<std::unique_ptr<Face>> faces;

public:

    DCEL();

    ~DCEL();

    /**
     * Initialize the figure given by the set of points
     * @param sortedPoints
     * @return The outer face of the generated polygon (Clock wise order face)
     */
    Face *initialize(const std::vector<std::vector<int>> &sortedPoints);

protected:
    /**
     * Get an edge that targets the vertex v, on the face f
     * @param v
     * @param f
     * @return
     */
    HalfEdge *getPreviousEdge(Vertex *v, Face *f);

    Face *getCommonFaceBetween(Vertex *v, Vertex *u);

public:

    const std::vector<std::unique_ptr<HalfEdge>> &getHalfEdges() {
        return this->halfEdges;
    }

    const std::vector<std::unique_ptr<Vertex>> &getVertexes() {
        return this->vertexes;
    }

    const std::vector<std::unique_ptr<Face>> &getFaces() {
        return this->faces;
    }

    /**
     * Add an edge between two vertices, effectively splitting a an existing face
     * @param v
     * @param u
     */
    Face *addEdge(Vertex *u, Vertex *v);

    /**
     * Add an edge between two vertices, effectively splitting an existing face
     * @param incidentOnU The half edge that is incident on the face we are splitting and who's target(incidentOnU) = u
     * @param v
     */
    Face *addEdge(HalfEdge *incidentOnU, Vertex *v);

    /**
     * Add a vertex to the DCEL structure, one with the coords
     * given by the vector and connected to the vertex that is
     * the target of h
     *
     * @param h
     * @returns A pointer to the new created vertex and the half edge that is incident on it
     */
    std::tuple<Vertex *, HalfEdge *> addVertex(const std::vector<int> &, HalfEdge *h);

    /**
     * Join the faces that are separated by the half edge h and it's
     * twin
     * @param h
     */
    void joinFaces(HalfEdge *h);

    /**
     * Split an edge by adding a vertex at the given coordinates
     *
     * @param h
     */
    void splitEdge(const std::vector<int> &, HalfEdge *h);

    void print();

    std::unique_ptr<std::string> serialize();

private:
    Face *addEdgeSingleFace(HalfEdge *incidentOnU, Vertex *v);

    Face *addEdgeClosedPolygon(HalfEdge *incidentOnU, Vertex *v);

    HalfEdge *getEdgeStartingIn(Vertex *v, Face *f);
};


#endif //TRABALHO2_DCEL_H
