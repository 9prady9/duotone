#include "face.h"
#include "id_generator.h"
#include "edge.h"
#include "vertex.h"
#include <sstream>
#include <vector>



Face::Face() {
  id_ = IdGenerator::getNextId(FACE);
}

std::vector<Edge*> Face::GetEdges() {
  return edges_;
}

int Face::SetEdges(std::vector<Edge*> edges) {
  edges_ = edges;
  return edges_.size();
}

std::vector<Vertex*> Face::GetVertices() {
  return vertices_;
}

int Face::SetVertices(std::vector<Vertex*> vertices) {
  vertices_ = vertices;
  return vertices_.size();
}

std::string Face::PrintEdges() {
  std::ostringstream os;
  for (unsigned int i = 0; i < edges_.size(); ++i) {
    if (i > 0) os << ",";
    os << edges_[i]->GetID();
  }
  return os.str();
}
std::string Face::PrintVertices() {
  std::ostringstream os;
  for (unsigned int i = 0; i < vertices_.size(); ++i) {
    if (i > 0) os << ",";
    os << vertices_[i]->GetID();
  }
  return os.str();
}
