#include "edge.h"

#include "id_generator.h"
#include "object_store.h"
#include "vertex.h"

Edge::Edge() {
  id_ = IdGenerator::getNextId(EDGE);
}

void Edge::SetVertices(Vertex* start, Vertex* end) {
  start_ = start;
  end_ = end;
}

void Edge::SetStart(Vertex* start) {
  start_ = start;
}

void Edge::SetEnd(Vertex* end) {
  end_ = end;
}

Vertex* Edge::GetStart() {
  return start_;
}

Vertex* Edge::GetEnd() {
  return end_;
}

Vertex* Edge::GetOtherEnd(Vertex* u) {
  if (u == start_) return end_;
  else return start_;
}
