/*
 * object_store.h
 *
 *  Created on: Jun 6, 2009
 *      Author: fhzhang
 */

#ifndef OBJECT_STORE_H_
#define OBJECT_STORE_H_

#include <vector>

class Edge;
class Face;
class Vertex;

/**
 * This class acts both as a factory and storage for Vertex and Edge objects.
 *
 * One shouldn't create or delete Vertex or Edge object directly.
 *
 * TODO: Find out if we can override the access for new and delete operator.
 * TODO: Make the constructors protected and this class friends of those two
 * classes.
 */
class ObjectStore {
public:
  // Return a free Vertex pointer or a new one.
  Vertex* CreateVertex();
  // Return a free Face pointer or a new one.
  Face* CreateFace();
  // Return a free Edge pointer or a new one.
  Edge* CreateEdge();
  Edge* CreateEdge(Vertex* start, Vertex* end);

  bool DeleteVertex(Vertex* v);
  bool DeleteEdge(Edge* e);
  bool DeleteFace(Face* f);

  // Clean up the vectors, remove the deleted items, free the memory.
  bool Compress();

  int GetFreeVerticesSize();
  int GetFreeEdgesSize();
  int GetFreeFacesSize();

  static ObjectStore* GetInstance();
private:
  // Make the constructor private so nobody can call it.
  ObjectStore();
  static ObjectStore* instance_;

  std::vector<Edge*> free_edges_;
  std::vector<Face*> free_faces_;
  std::vector<Vertex*> free_vertices_;
};

#endif /* OBJECT_STORE_H_ */
