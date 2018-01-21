#ifndef TOPOLOGY_VERTEX__
#define TOPOLOGY_VERTEX__

#include <map>
#include <string>
#include <vector>

#include "identifiable.h"

class Edge;
class ObjectStore;
// Vertex class.

// TODO: We need a means to guarantee that the edge ids we hold are all valid.
// TODO: Change the rotation into a list of edges to support loops and
// multiple-edges, - in a different version.

class Vertex : public Identifiable {
public:
  std::vector<Edge*> GetRotation();
  /* Returns true if successful, false if e_pre is invalid. */
  bool InsertEdgeInRotation(Edge* e, Edge* e_pre);

  /* for loading .obj file */
  void InsertEdgeInRotation_load(Edge* e, Edge* e_pre);

  /* Removes an edge incident at this vertex. */
  void RemoveEdgeFromRotation(Edge* e);

  /* Removes all edges incident at this vertex. */
  void ClearRotations();

  // Returns the next edge (after e_u).
  Edge* GetNextEdgeInRotation(Edge* e_u);

  // Returns the previous edge (before e_u).
  Edge* GetPreviousEdgeInRotation(Edge* e_u);

  // Returns the x_next edge of e_u
  Edge* Get_X_NextEdgeInRotation(Edge* e_u, int x);

  // Print the rotation at this vertex.
  std::string PrintRotation();

protected:
  // Hide the constructor so that only ObjectStore can create a vertex.
  Vertex();
  friend class ObjectStore;
  // We need a list to support loops.
  // Let's worry about loops later.
  std::map<Edge*, Edge*> rotation_;		// <edge, edge_next_in_rotation>
  // To make the remove operation efficient.
  std::map<Edge*, Edge*> reversed_rotation_;	// <edge, edge_pre_in_rotation>
};

#endif // TOPOLOGY_VERTEX__
