#ifndef GEOMETRY_RENDERABLE_OBJECT__
#define GEOMETRY_RENDERABLE_OBJECT__

#include "topology_object.h"

#include <map>
#include <vector>

class Face;
class Vertex;
/**
 * For regular rendering, we need
 *  (1) coordinates of each vertex
 *  (2) list of faces, each of them has a list of vertices (corners)
 *  (3) the normal of each face for planar modeling
 *  (4) the normal of each corner for polygonal modeling
 *  (5) the material of the object, or each face
 *  (6) color of the object, or each face
 *  (7) texture
 *
 * Many of these do not need to be stored with faces or vertices. We
 * can use property maps instead. These non-core properties are not
 * critical, i.e., they can be replace or reconstructed easily.
 *
 * The Topological Object keeps tracks of the core component
 *  (1) vertices (2) edges (3) rotations
 *    --- Good!
 * OR
 *  (1) vertices (2) faces with vertex-lists.
 *    --- not good enough, we could have multiple edges between vertices.
 * OR
 *  DLFL --- the current presentation is bad, but can be fixed.
 *
 * Hence it needs
 *  (1) add/delete vertices
 *  (2) add/delete edges
 */

// Object class.
class RenderableObject : public TopologyObject {
public:
  RenderableObject();

  std::set<Face*> GetFaces();
  float* GetVertexCoordinates(Vertex* v);
  float* GetFaceNormal(Face* f);
  void ReComputeFaces();

  bool SetCoords(Vertex* v, float*);

protected:
  std::set<Face*> faces_;
  // property map to store coordinates.
  // vertex_ID -> coordinates.
  // TODO: when the vertex is being removed, we have to delete coords pointers.
  std::map<Vertex*, float*> coords_;
  // face_ID -> normal.
  // TODO: when the face is being removed, we have to delete normal pointers.
  std::map<Face*, float*> normals_;
  // vertex_ID -> vertex_ID -> face_ID.
  std::map<Vertex*, std::map<Vertex*, Face*> > face_map_;
  // TODO: We need to know if the faces are up to date.
};

#endif // GEOMETRY_RENDERABLE_OBJECT__
