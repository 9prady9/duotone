#include "topology_object.h"
#include "logging.h"
#include "object_store.h"
#include "vertex.h"
#include "edge.h"

TopologyObject::~TopologyObject() {
  for (std::set<Vertex*>::iterator it = vertices_.begin();
       it != vertices_.end(); ++it) {
    ObjectStore::GetInstance()->DeleteVertex(*it);
  }
  for (std::set<Edge*>::iterator it = edges_.begin();
       it != edges_.end(); ++it) {
    ObjectStore::GetInstance()->DeleteEdge(*it);
  }
}

Vertex* TopologyObject::AddVertex() {
  Vertex* v = ObjectStore::GetInstance()->CreateVertex();
  vertices_.insert(v);
  LOGLINE("AddVertex " << v->GetID());
  return v;
}

bool TopologyObject::RemoveVertex(Vertex* vertex, bool force) {
  if (vertex->GetRotation().size() > 0) {
    if (!force) {
      return false;
    } else {
      // Remove all edges.
      std::vector<Edge*> edges = vertex->GetRotation();
      for (unsigned int ii = 0; ii < edges.size(); ++ii) {
        RemoveEdge(edges[ii]);
      }
    }
  }
  ObjectStore::GetInstance()->DeleteVertex(vertex);
  vertices_.erase(vertex);
  LOGLINE("RemoveVertex " << vertex->GetID());
  return true;
}

Edge* TopologyObject::AddEdge(Vertex* u, Vertex* v, Edge* e_u, Edge* e_v) {
  // How about thread safety?
  if (u == v || u == NULL || v == NULL) {
    return NULL;
  }
  Edge* e = ObjectStore::GetInstance()->CreateEdge(u, v);
  if (u->InsertEdgeInRotation(e, e_u)) {
    if (v->InsertEdgeInRotation(e, e_v)) {
      edges_.insert(e);
      LOGLINE("AddEdge " << e->GetID() << ":"
                << u->GetID() << "[" << (e_u == NULL ? -1 : e_u->GetID()) << "] - "
                << v->GetID() << "[" << (e_v == NULL ? -1 : e_v->GetID()) << "]"
               );
      return e;
    } else {
      // Roll back the operation on u.
      u->RemoveEdgeFromRotation(e);
    }
  }
  LOGLINE("AddEdge failed "
            << u->GetID() << "[" << (e_u == NULL ? -1 : e_u->GetID()) << "] - "
            << v->GetID() << "[" << (e_v == NULL ? -1 : e_v->GetID()) << "]");
  return NULL;
}

bool TopologyObject::RemoveEdge(Edge* edge) {
  // Invalid edge.
  if (edge == NULL) {
    return false;
  }
  // Do the deletion.
  Vertex* u = edge->GetStart();
  Vertex* v = edge->GetEnd();
  u->RemoveEdgeFromRotation(edge);
  v->RemoveEdgeFromRotation(edge);
  edges_.erase(edge);
  LOGLINE("RemoveEdge " << edge->GetID());
  ObjectStore::GetInstance()->DeleteEdge(edge);
  return true;
}

std::set<Vertex*> TopologyObject::GetVertices() {
  return vertices_;
}

std::set<Edge*> TopologyObject::GetEdges() {
  return edges_;
}
