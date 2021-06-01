#include "dcel.h"
#include <utility>
#include <iostream>


DCEL::DCEL() : halfEdges(), faces(), vertexes() {

}

HalfEdge *DCEL::getPreviousEdge(Vertex *v, Face *f) {

    HalfEdge *twin = v->getHalfEdge()->getTwin(),
            *edge = twin->getNext()->getTwin();

    while (edge != twin) {
        if (edge->getIncidentFace() == f) {
            return edge;
        }

        edge = edge->getNext()->getTwin();
    }

    return edge;
}

Face *DCEL::getCommonFaceBetween(Vertex *v, Vertex *u) {

    if (v->getHalfEdge()->getIncidentFace() == u->getHalfEdge()->getIncidentFace()) {
        return v->getHalfEdge()->getIncidentFace();
    }

    auto edge1 = v->getHalfEdge()->getTwin()->getNext()->getTwin();

    while (edge1 != v->getHalfEdge()->getTwin()) {
        auto edge2 = u->getHalfEdge()->getTwin()->getNext()->getTwin();

        while (edge2 != u->getHalfEdge()->getTwin()) {

            if (edge1->getIncidentFace() == edge2->getIncidentFace()) {
                return edge1->getIncidentFace();
            }

            edge2 = edge2->getNext()->getTwin();
        }

        edge1 = edge1->getNext()->getTwin();
    }

    return v->getHalfEdge()->getIncidentFace();
}

void DCEL::initialize(const std::vector<std::vector<int>> &sortedPoints) {

    if (sortedPoints.size() < 2) {
        return;
    }

    //We start with 2 points united by an edge

    std::vector<int> p1 = sortedPoints[0], p2 = sortedPoints[1];

    auto edge1 = std::make_unique<HalfEdge>(), edge2 = std::make_unique<HalfEdge>();

    auto face1 = std::make_unique<Face>(edge1.get());

    edge1->setIncidentFace(face1.get());
    edge2->setIncidentFace(face1.get());

    edge1->setTwin(edge2.get());
    edge2->setTwin(edge1.get());

    auto vertex1 = std::make_unique<Vertex>(p1), vertex2 = std::make_unique<Vertex>(p2);

    edge1->setTargetVertex(vertex1.get());
    edge2->setTargetVertex(vertex2.get());

    vertex1->setHalfEdge(edge2.get());
    vertex2->setHalfEdge(edge1.get());

    Vertex *first = vertex1.get();

    HalfEdge *prev = edge2.get();

    this->faces.push_back(std::move(face1));
    this->halfEdges.push_back(std::move(edge1));
    this->halfEdges.push_back(std::move(edge2));
    this->vertexes.push_back(std::move(vertex2));
    this->vertexes.push_back(std::move(vertex1));

    for (int i = 2; i < sortedPoints.size(); i++) {
        auto added = addVertex(sortedPoints[i], prev);

        prev = std::get<1>(added);
    }

    addEdge(prev, first);
}

void DCEL::addEdge(Vertex *u, Vertex *v) {

    auto face = getCommonFaceBetween(u, v);

    addEdge(getPreviousEdge(u, face), v);
}

void DCEL::addEdge(HalfEdge *incidentOnU, Vertex *v) {

    if (incidentOnU->getIncidentFace() != incidentOnU->getTwin()->getIncidentFace()) {
        addEdgeClosedPolygon(incidentOnU, v);
    } else {
        addEdgeSingleFace(incidentOnU, v);
    }

}

void DCEL::addEdgeClosedPolygon(HalfEdge *incidentOnU, Vertex *v) {
    Vertex *u = incidentOnU->getTargetVertex();

    Face *f = incidentOnU->getIncidentFace();

    auto halfE1 = std::make_unique<HalfEdge>();
    auto halfE2 = std::make_unique<HalfEdge>();

    auto face1 = std::make_unique<Face>(halfE1.get());
    auto face2 = std::make_unique<Face>(halfE2.get());

    halfE1->setTwin(halfE2.get());
    halfE2->setTwin(halfE1.get());

    halfE1->setTargetVertex(v);
    halfE2->setTargetVertex(u);

    halfE2->setNext(incidentOnU->getNext());
    halfE2->getNext()->setPrevious(halfE2->getNext());
    halfE1->setPrevious(incidentOnU);
    incidentOnU->setNext(halfE1.get());

    HalfEdge* i = halfE2.get();

    while(true) {
        i->setIncidentFace(face2.get());

        if (i->getTargetVertex() == v) break;

        i = i->getNext();
    }

    halfE1->setNext(i->getNext());
    halfE1->getNext()->setPrevious(halfE1.get());

    i->setNext(halfE2.get());
    halfE2->setPrevious(i);

    i = halfE1.get();

    do {
        i->setIncidentFace(face1.get());
        i = i->getNext();
    } while (i->getTargetVertex() != u);


    for (auto it = this->faces.begin(); it != this->faces.end(); it++) {

        if ((*it).get() == f) {
            it = this->faces.erase(it);
            break;
        }
    }

    this->faces.push_back(std::move(face1));
    this->faces.push_back(std::move(face2));

    this->halfEdges.push_back(std::move(halfE1));
    this->halfEdges.push_back(std::move(halfE2));
}

void DCEL::addEdgeSingleFace(HalfEdge *incidentOnU, Vertex *v) {
    Vertex *u = incidentOnU->getTargetVertex();

    Face *f = incidentOnU->getIncidentFace();

    auto halfE1 = std::make_unique<HalfEdge>();
    auto halfE2 = std::make_unique<HalfEdge>();

    auto face1 = std::make_unique<Face>(halfE1.get());
    auto face2 = std::make_unique<Face>(halfE2.get());

    halfE2->setTwin(halfE1.get());
    halfE1->setTwin(halfE2.get());

    halfE1->setTargetVertex(v);
    halfE2->setTargetVertex(u);

    halfE2->setNext(incidentOnU->getTwin());
    halfE2->getNext()->setPrevious(halfE2.get());

    halfE1->setPrevious(incidentOnU);

    incidentOnU->setNext(halfE1.get());

    auto i = halfE2.get();

    while (true) {

        i->setIncidentFace(face2.get());

        if (i->getTargetVertex() == v) break;

        i = i->getNext();
    }

    i->setNext(halfE2.get());

    halfE1->setNext(i->getTwin());

    halfE1->getNext()->setPrevious(halfE1.get());

    halfE2->setPrevious(i);

    i = halfE1.get();

    do {

        i->setIncidentFace(face1.get());
        i = i->getNext();

    } while (i->getTargetVertex() != u);

    for (auto it = this->faces.begin(); it != this->faces.end(); it++) {

        if ((*it).get() == f) {
            it = this->faces.erase(it);
            break;
        }

    }

    this->faces.push_back(std::move(face1));
    this->faces.push_back(std::move(face2));

    this->halfEdges.push_back(std::move(halfE1));
    this->halfEdges.push_back(std::move(halfE2));
}

std::tuple<Vertex *, HalfEdge *> DCEL::addVertex(const std::vector<int> &coords, HalfEdge *h) {

    auto u = h->getTargetVertex();

    auto f = h->getIncidentFace();

    auto edge1 = std::make_unique<HalfEdge>();
    auto edge2 = std::make_unique<HalfEdge>();

    auto vertex = std::make_unique<Vertex>(coords);

    vertex->setHalfEdge(edge2.get());

    edge1->setTwin(edge2.get());
    edge2->setTwin(edge1.get());

    edge1->setTargetVertex(vertex.get());
    edge2->setTargetVertex(u);

    edge1->setIncidentFace(f);
    edge2->setIncidentFace(f);

    edge2->setNext(h->getTwin());

    edge1->setPrevious(h);

    h->setNext(edge1.get());

    edge2->getNext()->setPrevious(edge2.get());

    Vertex *v = vertex.get();
    HalfEdge *incidentOn = edge1.get();

    this->vertexes.push_back(std::move(vertex));

    this->halfEdges.push_back(std::move(edge1));
    this->halfEdges.push_back(std::move(edge2));

    return std::make_tuple(v, incidentOn);
}

void DCEL::joinFaces(HalfEdge *h) {

    h->getNext()->setPrevious(h->getTwin()->getPrevious());
    h->getTwin()->getNext()->setPrevious(h->getPrevious());

    h->getPrevious()->setNext(h->getTwin()->getNext());
    h->getTwin()->getPrevious()->setNext(h->getNext());

    auto face1 = h->getIncidentFace();
    auto face2 = h->getTwin()->getIncidentFace();

    auto face = std::make_unique<Face>(h->getNext());

    auto start = h->getNext();

    do {

        start->setIncidentFace(face.get());

        start = start->getNext();

    } while (start != h);

    start = h->getTwin()->getNext();

    do {
        start->setIncidentFace(face.get());

        start = start->getNext();
    } while (start != h->getTwin());

    for (auto it = this->halfEdges.begin(); it != this->halfEdges.end(); it++) {

        if ((*it).get() == h || (*it).get() == h->getTwin()) {
            it = this->halfEdges.erase(it);
        }

    }

    for (auto it = this->faces.begin(); it != this->faces.end(); it++) {

        if ((*it).get() == face1 || (*it).get() == face2) {
            it = this->faces.erase(it);
        }

    }

    this->faces.push_back(std::move(face));

}


void DCEL::splitEdge(const std::vector<int> &coords, HalfEdge *h) {

    auto vertex = std::make_unique<Vertex>(coords);

    auto nEdge1 = std::make_unique<HalfEdge>();

    auto nEdge2 = std::make_unique<HalfEdge>();

    nEdge1->setTwin(nEdge2.get());
    nEdge2->setTwin(nEdge1.get());

    nEdge1->setPrevious(h);

    nEdge1->setNext(h->getNext());

    nEdge1->setTargetVertex(h->getTargetVertex());

    nEdge1->setIncidentFace(h->getIncidentFace());

    h->setTargetVertex(vertex.get());

    h->setNext(nEdge1.get());

    auto twin = h->getTwin();

    nEdge2->setPrevious(twin->getPrevious());

    nEdge2->setNext(twin);

    nEdge2->setTargetVertex(vertex.get());

    nEdge2->setIncidentFace(twin->getIncidentFace());

    twin->setPrevious(nEdge2.get());

    this->halfEdges.push_back(std::move(nEdge1));
    this->halfEdges.push_back(std::move(nEdge2));

    this->vertexes.push_back(std::move(vertex));
}

void DCEL::print() {

    Vertex *vertex = this->vertexes[0].get();

    HalfEdge *edge = vertex->getHalfEdge();
    HalfEdge *first = edge;

    do {

        std::cout << edge->getTargetVertex()->point()[0] << " " << edge->getTargetVertex()->point()[1] << std::endl;

        edge = edge->getNext();
    } while (edge != first);

    std::cout << "Face count: " << this->faces.size() << std::endl;

    std::cout << "Vertex count: " << this->vertexes.size() << std::endl;

    std::cout << "Half Edges count: " << this->halfEdges.size() << std::endl;

}


