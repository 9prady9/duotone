#include "vertex.h"
#include "identifiable.h"
#include "id_generator.h"
#include "edge.h"

#include <sstream>

Vertex::Vertex() {
  id_ = IdGenerator::getNextId(VERTEX);
}

std::vector<Edge*> Vertex::GetRotation() {
  std::vector<Edge*> rotation;
  for (std::map<Edge*, Edge*>::iterator it = rotation_.begin();
      it != rotation_.end(); ++it) {
    rotation.push_back(it->first);
  }
  return rotation;
}

// TODO: Need to make sure the code is consistent. The ideal case is that we
//       always create edge between two vertices and tell where we should insert
//       them in the rotation system. This function should belong (or only
//       be accessible from) to a higher
//       level class. -- Move it and the RemoveEdge to TopoObj level.
// TODO: if e is a loop, there will be a bug in the rotation. Fix it!!!
bool Vertex::InsertEdgeInRotation(Edge* e, Edge* e_pre) {
  if (e->GetStart() != this && e->GetEnd() != this) {
    return false;
  }
  if (rotation_.size() <= 0) {
    rotation_[e] = e;
    reversed_rotation_[e] = e;
    return true;
  }
  if (rotation_.find(e_pre) == rotation_.end() ||
      rotation_.find(e) != rotation_.end()) {
    return false;
  }
  Edge* e_next = rotation_[e_pre];
  rotation_[e_pre] = e;
  rotation_[e] = e_next;
  reversed_rotation_[e_next] = e;
  reversed_rotation_[e] = e_pre;
  return true;
}

void Vertex::InsertEdgeInRotation_load(Edge* e, Edge* e_pre) {
    rotation_[e_pre] = e;
    reversed_rotation_[e] = e_pre;
}


void Vertex::RemoveEdgeFromRotation(Edge* e) {
  Edge* e_pre = reversed_rotation_[e];
  Edge* e_next = rotation_[e];
  if (e_pre != e && e_next != e) {
    rotation_[e_pre] = e_next;
    reversed_rotation_[e_next] = e_pre;
  }
  rotation_.erase(e);
  reversed_rotation_.erase(e);
}

Edge* Vertex::GetNextEdgeInRotation(Edge* e_u) {
  if (rotation_.find(e_u) == rotation_.end()) return NULL;
  return rotation_[e_u];
}

Edge* Vertex::GetPreviousEdgeInRotation(Edge* e_u) {
  if (reversed_rotation_.find(e_u) == reversed_rotation_.end()) return NULL;
  return reversed_rotation_[e_u];
}

// Returns the x_next edge of e_u
Edge* Vertex::Get_X_NextEdgeInRotation(Edge* e_u, int x) {
    Edge* x_next_edge = NULL;
    switch ( x )
    {
    case 0:		// itself
        x_next_edge = e_u;
        break;

    case 1:		// next
        x_next_edge = GetNextEdgeInRotation(e_u);
        break;

    case -1:	// previous
        x_next_edge = GetPreviousEdgeInRotation(e_u);
        break;

    default:
        break;
    }
    return x_next_edge;
}

void Vertex::ClearRotations() {
  rotation_.clear();
  reversed_rotation_.clear();
}

std::string Vertex::PrintRotation() {
  std::ostringstream os;
  Edge* e = rotation_.begin()->second;
  while (e != rotation_.begin()->first) {
    os << e->GetID() << "[" << e->GetOtherEnd(this)->GetID() << "]->";
    e = rotation_[e];
  }
  os << e->GetID() << "[" << e->GetOtherEnd(this)->GetID() << "]";
  return os.str();
}
