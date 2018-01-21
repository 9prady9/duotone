/*
 * object_store.cc
 *
 *  Created on: Jun 6, 2009
 *      Author: fhzhang
 */
// #include "Fenghui_Zhang_core/common/object_store.h"
// 
// #include "Fenghui_Zhang_core/topology/vertex.h"
// #include "Fenghui_Zhang_core/topology/edge.h"
// #include "Fenghui_Zhang_core/topology/face.h"

#include "object_store.h"
#include "vertex.h"
#include "edge.h"
#include "face.h"


// Note: This class is not thread-safe!!!
// TODO: add thread safety guard.

ObjectStore* ObjectStore::instance_ = NULL;

ObjectStore::ObjectStore() {}

/*static*/ ObjectStore* ObjectStore::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new ObjectStore();
  }
  return instance_;
}

Vertex* ObjectStore::CreateVertex() {
  if (free_vertices_.size() > 0) {
    Vertex* v = free_vertices_.back();
    free_vertices_.pop_back();
    return v;
  }
  Vertex* v = new Vertex();
  return v;
}

Edge* ObjectStore::CreateEdge(Vertex* start, Vertex* end) {
  Edge* e = CreateEdge();
  e->SetVertices(start, end);
  return e;
}

Edge* ObjectStore::CreateEdge() {
  if (free_edges_.size() > 0) {
    Edge* e = free_edges_.back();
    free_edges_.pop_back();
    return e;
  }
  Edge* e = new Edge();
  return e;
}

Face* ObjectStore::CreateFace() {
  if (free_faces_.size() > 0) {
    Face* f = free_faces_.back();
    free_faces_.pop_back();
    return f;
  }
  Face* f = new Face();
  return f;
}


bool ObjectStore::DeleteVertex(Vertex* v) {
  free_vertices_.push_back(v);
  return true;
}

bool ObjectStore::DeleteEdge(Edge* e) {
  free_edges_.push_back(e);
  return true;
}

bool ObjectStore::DeleteFace(Face* f) {
  free_faces_.push_back(f);
  return true;
}

// Clean up the vectors, remove the deleted items, free the memory.
bool ObjectStore::Compress() {
  for (unsigned int i = 0; i < free_vertices_.size(); ++i) {
    delete free_vertices_[i];
  }
  free_vertices_.clear();
  for (unsigned int i = 0; i < free_edges_.size(); ++i) {
    delete free_edges_[i];
  }
  free_edges_.clear();
  for (unsigned int i = 0; i < free_faces_.size (); ++i ) {
      delete free_faces_[i];
  }
  free_faces_.clear ();
  return true;
}

int ObjectStore::GetFreeVerticesSize() {
  return free_vertices_.size();
}

int ObjectStore::GetFreeEdgesSize() {
  return free_edges_.size();
}

int ObjectStore::GetFreeFacesSize() {
  return free_faces_.size();
}
