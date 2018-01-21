#ifndef TOPOLOGY_EDGE__
#define TOPOLOGY_EDGE__

#include "identifiable.h"

#include <vector>

class Vertex;
class ObjectStore;
// Edge class.
/*
 * An edge knows it's two endpoints (vertices).
 *
 */
class Edge : public Identifiable {
public:
  // Direction property that can be attached to an edge.
  // NONE for undirected edge. A face need to know the direction of its edges.

  // If we use a propertymap, the property will be separated from the Edge.
  // But it will save us space.
  enum Direction { NONE, REVERSED, DIRECTED };

  void SetVertices(Vertex* start, Vertex* end);
  void SetStart(Vertex* start);
  void SetEnd(Vertex* end);

  Vertex* GetStart();
  Vertex* GetEnd();

  Vertex* GetOtherEnd(Vertex* u);
protected:
  // Make the constructor protected so that it can't be created.
  Edge();

  friend class ObjectStore;
  // Vertex with smaller ID.
  Vertex* start_;
  // Vertex with larger ID.
  Vertex* end_;
};

#endif // TOPOLOGY_EDGE__
