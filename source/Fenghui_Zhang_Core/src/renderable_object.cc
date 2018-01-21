#include "renderable_object.h"
#include "logging.h"
#include "object_store.h"
#include "edge.h"
#include "face.h"
#include "vertex.h"



// Multiplication - cross product
void CrossProduct(const float* u, const float* v, float p[]) {
  p[0] = u[1]*v[2] - u[2]*v[1];
  p[1] = u[2]*v[0] - u[0]*v[2];
  p[2] = u[0]*v[1] - u[1]*v[0];
}

/**
 * Possible problems with normals:
 *
 *   (1) Face not planar.
 *      Need normal at each corner
 *   (2) Corner is winged (0 degree or 180)
 *      Need to find the nearest nonWinged corner
 *   (3) Face in a straight line.
 *      Identify it and ignore it.
 *
 * It's easier just adding all the normals of the corners up, in that case we
 * don't have to worry about if it is planar or not.
 *
 */
float* RenderableObject::GetFaceNormal(Face* face) {
  if (normals_.find(face) == normals_.end()) {
    // Compute normal for the face.
    float U[3] = {0.0, 0.0, 0.0};
    float V[3] = {0.0, 0.0, 0.0};
    float *u_1 = GetVertexCoordinates(face->GetVertices()[0]);
    float *u_2 = GetVertexCoordinates(face->GetVertices()[1]);
    float *u_3 = GetVertexCoordinates(face->GetVertices()[2]);
    U[0] = u_2[0] - u_1[0];
    U[1] = u_2[1] - u_1[1];
    U[2] = u_2[2] - u_1[2];

    V[0] = u_3[0] - u_2[0];
    V[1] = u_3[1] - u_2[1];
    V[2] = u_3[2] - u_2[2];

    float *normal = new float[3];
    CrossProduct(U, V, normal);

    normals_[face] = normal;
  }
  return normals_[face];
}

RenderableObject::RenderableObject() {}

// Capture the ownership of the coords pointer.
// TODO: release the pointer when it's not needed any more, basically when
//   purge the vertex.
bool RenderableObject::SetCoords(Vertex* v, float* coords) {
  if (v == NULL) return false;
  coords_[v] = coords;
  LOGLINE("set position:" << v->GetID()
          << "[" << coords[0] << "," << coords[1] << "," << coords[2] << "]");
  return true;
}

float * RenderableObject::GetVertexCoordinates(Vertex* v){
  if (v == NULL) return NULL;
  return coords_[v];
}

std::set<Face*> RenderableObject::GetFaces() {
  return faces_;
}

// Traces all the faces and add them to faces_.
void RenderableObject::ReComputeFaces() {
  for (std::set<Face*>::iterator it = faces_.begin();
      it != faces_.end(); ++it) {
    ObjectStore::GetInstance()->DeleteFace(*it);
  }
  faces_.clear();
  face_map_.clear();

  std::set<Edge*> visited_edges;
  std::set<Edge*> edges;
  for (std::set<Edge*>::iterator it = edges_.begin();
      it != edges_.end(); ++it) {
    edges.insert((*it));
  }
  while(!edges.empty()) {
    Edge* e = *(edges.begin());
    Vertex* u = e->GetStart();
    Vertex* v = e->GetEnd();
    // Make sure u->v is the half edge we haven't visited yet.
    if (face_map_.find(u) != face_map_.end() &&
        face_map_[u].find(v) != face_map_[u].end()) {
      std::swap(u, v);
    }
    // Trace face and collect vertices along the way.
    Face* new_face = ObjectStore::GetInstance()->CreateFace();
    std::vector<Vertex*> face_vertices;
    std::vector<Edge*> face_edges;
    Vertex* s = u;
    Vertex* t = v;
    Edge* e_current = e;
    do {
      if (visited_edges.find(e_current) != visited_edges.end()) {
        edges.erase(e_current);
      } else {
        visited_edges.insert(e_current);
      }
      face_vertices.push_back(s);
      face_edges.push_back(e_current);
      // The half edge s->t belongs to new_face now.
      face_map_[s][t] = new_face;
      // Next edge in rotation after e_current.
      Edge* e_next = t->GetNextEdgeInRotation(e_current);
      // Next vertex after v.
      Vertex* w = e_next->GetOtherEnd(t);
      e_current = e_next;
      s = t;
      t = w;
    } while(s != u || t != v || e_current != e);
    new_face->SetVertices(face_vertices);
    new_face->SetEdges(face_edges);
    faces_.insert(new_face);
  }
}
