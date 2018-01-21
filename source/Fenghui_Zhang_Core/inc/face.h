#ifndef TOPOLOGY_FACE__
#define TOPOLOGY_FACE__

#include "identifiable.h"

#include <string>
#include <vector>

class Edge;
class Vertex;
class ObjectStore;
// Face class.
/*
 * Note: we tend not to store faces in the core object.
 *
 * To be safe, face should be a series of directed edges.
 */
class Face : public Identifiable {
public:
  std::vector<Vertex*> GetVertices();
  std::vector<Edge*> GetEdges();
  int SetVertices(std::vector<Vertex*> vertices);
  int SetEdges(std::vector<Edge*> edges);

  std::string PrintEdges();
  std::string PrintVertices();

protected:
  Face();
  friend class ObjectStore;
  std::vector<Vertex*> vertices_;
  std::vector<Edge*> edges_;
};

#endif // TOPOLOGY_FACE__
