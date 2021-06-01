#ifndef TRABALHO2_DCEL_H
#define TRABALHO2_DCEL_H

#include <vector>
#include <memory>

class Vertex;
class Face;

class HalfEdge {

private:
    Vertex *targetVertex;

    Face *incidentFace;

    HalfEdge *twin, *next, *previous;

public:
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
    Vertex(std::vector<int> coordinates) :
            coordinates(std::move(coordinates)) {}

public:
    HalfEdge *getHalfEdge() const {
        return halfEdge;
    }

    const std::vector<int> &point() const {
        return coordinates;
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
};

/**
 * Doubly connected edge list
 */
class DCEL {

private:
    std::vector<std::unique_ptr<HalfEdge>> halfEdges;

    std::vector<std::unique_ptr<Vertex>> vertexes;

    std::vector<std::unique_ptr<Face>> faces;

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

    DCEL();

    void initialize(const std::vector<std::vector<int>> &sortedPoints);

    /**
     * Add an edge between two vertices, effectively splitting a an existing face
     * @param v
     * @param u
     */
    void addEdge(Vertex *u, Vertex *v);

    /**
     * Add an edge between two vertices, effectively splitting an existing face
     * @param incidentOnU The half edge that is incident on the face we are splitting and who's target(incidentOnU) = u
     * @param v
     */
    void addEdge(HalfEdge *incidentOnU, Vertex *v);

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
private:
    void addEdgeSingleFace(HalfEdge *incidentOnU, Vertex *v);

    void addEdgeClosedPolygon(HalfEdge *incidentOnU, Vertex *v);
};

#endif //TRABALHO2_DCEL_H
