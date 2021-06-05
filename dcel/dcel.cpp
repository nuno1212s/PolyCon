#include "dcel.h"
#include <utility>
#include <iostream>

DCEL::DCEL() : halfEdges(), faces(), vertexes() {

}

DCEL::~DCEL() {


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

/**
 * We know that there can only be 1 half edge starting at v for a given face f
 * @param v
 * @param f
 * @return
 */
HalfEdge *DCEL::getEdgeStartingIn(Vertex *v, Face *f) {

}

Face *DCEL::initialize(const std::vector<std::vector<int>> &sortedPoints) {

    if (sortedPoints.size() < 2) {
        return nullptr;
    }

    if (!this->faces.empty()) {
        std::cerr << "DCEL is already initialized" << std::endl;

        return nullptr;
    }

    //We start with 2 points united by an edge

    const std::vector<int> &p1 = sortedPoints[0], &p2 = sortedPoints[1];

    auto edge1 = std::make_unique<HalfEdge>(), edge2 = std::make_unique<HalfEdge>();

    auto face1 = std::make_unique<Face>(edge1.get());

    edge1->setIncidentFace(face1.get());
    edge2->setIncidentFace(face1.get());

    edge1->setTwin(edge2.get());
    edge2->setTwin(edge1.get());

    auto vertex1 = std::make_unique<Vertex>(p1), vertex2 = std::make_unique<Vertex>(p2);

    edge1->setTargetVertex(vertex1.get());
    edge1->setOriginVertex(vertex2.get());

    edge2->setTargetVertex(vertex2.get());
    edge2->setOriginVertex(vertex1.get());

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

    Face *out = addEdge(prev, first);

    traverseFace(out);

    return out;
}

Face *DCEL::addEdge(Vertex *u, Vertex *v) {
    auto face = getCommonFaceBetween(u, v);

    return addEdge(getPreviousEdge(u, face), v);
}

Face *DCEL::addEdge(HalfEdge *incidentOnU, Vertex *v) {

    if (incidentOnU->getIncidentFace() != incidentOnU->getTwin()->getIncidentFace()) {
        return addEdgeClosedPolygon(incidentOnU, v);
    } else {
        return addEdgeSingleFace(incidentOnU, v);
    }
}

Face *DCEL::addEdgeClosedPolygon(HalfEdge *incidentOnU, Vertex *v) {
    Vertex *u = incidentOnU->getTargetVertex();

    Face *f = incidentOnU->getIncidentFace();

    auto halfE1 = std::make_unique<HalfEdge>();
    auto halfE2 = std::make_unique<HalfEdge>();

    auto face1 = std::make_unique<Face>(halfE1.get());
    auto face2 = std::make_unique<Face>(halfE2.get());

    halfE1->setTwin(halfE2.get());
    halfE2->setTwin(halfE1.get());

    //H1 goes from u to v
    halfE1->setTargetVertex(v);
    halfE1->setOriginVertex(u);

    //H2 goes from v to u
    halfE2->setTargetVertex(u);
    halfE2->setOriginVertex(v);

    halfE2->setNext(incidentOnU->getNext());
    halfE2->getNext()->setPrevious(halfE2->getNext());
    halfE1->setPrevious(incidentOnU);
    incidentOnU->setNext(halfE1.get());

    HalfEdge *i = halfE2.get();

    while (true) {
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

    Face *outsideFace = face1.get();

    this->faces.push_back(std::move(face1));
    this->faces.push_back(std::move(face2));

    this->halfEdges.push_back(std::move(halfE1));
    this->halfEdges.push_back(std::move(halfE2));

    return outsideFace;
}

Face *DCEL::addEdgeSingleFace(HalfEdge *incidentOnU, Vertex *v) {
    Vertex *u = incidentOnU->getTargetVertex();

    Face *f = incidentOnU->getIncidentFace();

    auto halfE1 = std::make_unique<HalfEdge>();
    auto halfE2 = std::make_unique<HalfEdge>();

    auto face1 = std::make_unique<Face>(halfE1.get());
    auto face2 = std::make_unique<Face>(halfE2.get());

    halfE2->setTwin(halfE1.get());
    halfE1->setTwin(halfE2.get());

    //H1 goes from u to v
    halfE1->setOriginVertex(u);
    halfE1->setTargetVertex(v);

    //H2 goes from v to u
    halfE2->setOriginVertex(v);
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

    halfE2->setPrevious(i);

    bool result = false;

    if (i->getTwin()->getIncidentFace() != f) {
        //This means that this edge is not part of the new face we are creating

        HalfEdge *start = i;

        do {
            start = start->getTwin();

            while (start->getTargetVertex() != v) {
                start = start->getNext();
            }
        } while (start->getTwin()->getIncidentFace() != f);

        //This should mean that start.getTwin().getIncidentFace == f
        halfE1->setNext(start->getTwin());

        result = true;
    } else {
        halfE1->setNext(i->getTwin());
    }

    halfE1->getNext()->setPrevious(halfE1.get());

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

    Face *outsideFace = nullptr;

    if (result) {
        outsideFace = halfE2->getIncidentFace();
    } else {
        outsideFace = halfE1->getIncidentFace();
    }

    this->faces.push_back(std::move(face1));
    this->faces.push_back(std::move(face2));

    this->halfEdges.push_back(std::move(halfE1));
    this->halfEdges.push_back(std::move(halfE2));

    return outsideFace;
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

    edge2->setOriginVertex(u);
    edge2->setTargetVertex(vertex.get());

    edge1->setOriginVertex(vertex.get());
    edge1->setTargetVertex(u);

    edge1->setIncidentFace(f);
    edge2->setIncidentFace(f);

    if (h->getTwin()->getIncidentFace() != f) {
        std::cout << "not incident " << std::endl;
        edge1->setNext(h->getNext());
    } else {
        std::cout << "incident " << std::endl;
        edge1->setNext(h->getTwin());
    }

    if (edge1->getNext() != nullptr)
        edge1->getNext()->setPrevious(edge1.get());

    edge2->setPrevious(h);
    h->setNext(edge2.get());

    Vertex *v = vertex.get();
    HalfEdge *incidentOn = edge2.get();

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

    std::cout << "Face count: " << this->faces.size() << std::endl;

    int count = 0;

    for (auto const &face : this->faces) {

        std::cout << "Face nr: " << count++ << std::endl;

        HalfEdge *edge = face->getArbitraryHalfEdge(), *first = edge;

        do {

            Vertex *v = edge->getTargetVertex();

            std::cout << v->point()[0] << " " << v->point()[1] << std::endl;

            edge = edge->getNext();

        } while (edge != first);

    }

    std::cout << "Vertex count: " << this->vertexes.size() << std::endl;

    std::cout << "Half Edges count: " << this->halfEdges.size() << std::endl;
}


