#ifndef TOPOLOGY_OBJECT__
#define TOPOLOGY_OBJECT__

#include <map>
#include <set>
#include <vector>

class Vertex;
class Edge;
/*
 * Topology object class.
 *
 * We don't allow loops and multiple edges for now. Shortly, we only support
 * simple graphs.
 *
 * The problem with loops is rotation, when two half edges of the loop are next
 * to each other. Our map storage in the vertex won't make it.
 *
 * The problem with multiple edge is the vertices->edge map. This might not be
 * a problem, but we will have to see.
 *
 * Do we need to know which edge is between which vertex and which?
 */

// TopologyObject class.
class TopologyObject {
public:
  ~TopologyObject();
  // Add a new vertex, return the pointer.
  Vertex* AddVertex();
  // Remove a vertex, and all edges incident at it if force is true.
  // If force is false and there are edges incident at
  // the vertex, return false.
  // If force is true, remove all edges first. Then remove the vertex and return true.
  bool RemoveVertex(Vertex* vertex, bool force);

  std::set<Vertex*> GetVertices();
  std::set<Edge*> GetEdges();

  // Add an edge between u and v, the edge will be inserted after
  // edge e_u, and e_v in their rotations.
  // if e_u(e_v) is invalid, this edge is the first edge incident at u(v).
  Edge* AddEdge(Vertex* u, Vertex* v, Edge* e_u, Edge* e_v);

  // Removes the edge with given pointer. Returns false if pointer is invalid.
  bool RemoveEdge(Edge* edge);

  // We didn't find use of these two methods for now, so we can have multiple
  // edges for now.
  /*
  int GetEdge(int u, int v);
  int GetEdge(Vertex u, Vertex v);
  */

protected:
  std::set<Vertex*> vertices_;
  std::set<Edge*> edges_;

  // Don't need the following for now.
  // This map is to help find which vertex is connected to which edge.
  // Works for simple graph. Might not be good enough for graph that
  // allows multiple edges.
  // vertexID -> vertexID -> EdgeID.
  /*
  std::map<int, std::map<int, int> > edge_map_;
  */
};

#endif // TOPOLOGY_OBJECT__
