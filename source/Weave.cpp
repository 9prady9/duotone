#include "Weave.h"
#include "topology_object.h"
#include "face.h"
#include "edge.h"
#include "vertex.h"
#include "logging.h"
#include "object_store.h"
#include "vector3d.h"

#include <map>
#include <ctime>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <utility>
#include <string.h>
#include <iostream>
#include <algorithm>
#include <cinder\TriMesh.h>
#include <boost\foreach.hpp>
#include <boost\tokenizer.hpp>

using namespace std;
using namespace ci;
using namespace boost;

Tile::Tile(unsigned int num_vertices) {
    n = num_vertices;
    marked = -1;
    knotType=PLUS1;
    //knotType=TWIST1;
    visited=0; isCrv1Used=NO; isCrv2Used = NO;
}

Tile::~Tile() { up.clear(); down.clear(); }

Vector3D Tile::trilinearInterpolation(float u,float v,float t)
{
    Vector3D ret_val(0.0,0.0,0.0);
    ret_val += ( ((1-u)*(1-v)*(1-t))*(*up[0]) );
    ret_val += ( (u*(1-v)*(1-t))*(*up[1]) );
    ret_val += ( (u*v*(1-t))*(*up[2]) );
    ret_val += ( ((1-u)*v*(1-t))*(*up[3]) );

    ret_val += ( ((1-u)*(1-v)*t)*(*down[0]) );
    ret_val += ( (u*(1-v)*t)*(*down[1]) );
    ret_val += ( (u*v*t)*(*down[2]) );
    ret_val += ( ((1-u)*v*t)*(*down[3]) );
    return ret_val;
}

void Tile::generateCurvePoints()
{
    /* Safety purposes clear both vector of points */
    curve1.clear();
    curve2.clear();
    Vector3D *tmp;
    tmp = new Vector3D(trilinearInterpolation(0.5,0.25,0.5));
    curve1.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.5,0.5,0.75));
    curve1.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.5,0.75,0.5));
    curve1.push_back(tmp);

    tmp = new Vector3D(trilinearInterpolation(0.25,0.5,0.5));
    curve2.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.5,0.5,0.25));
    curve2.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.75,0.5,0.5));
    curve2.push_back(tmp);
}

void Tile::generateTwist1Points()
{
    /* A call to this function clears out any curve points
     *generated for PLUS1/PLUS2 with new twisted curves points */
    curve1.clear();
    curve2.clear();
    Vector3D *tmp;
    tmp = new Vector3D(trilinearInterpolation(0.5,0.25,0.75));
    curve1.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.75,0.5,0.5));
    curve1.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.25,0.75,0.25));
    curve1.push_back(tmp);

    tmp = new Vector3D(trilinearInterpolation(0.75,0.25,0.25));
    curve2.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.25,0.5,0.5));
    curve2.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.5,0.75,0.75));
    curve2.push_back(tmp);
}

void Tile::generateTwist2Points()
{
    /* A call to this function clears out any curve points
     *generated for PLUS1/PLUS2 with new twisted curves points */
    curve1.clear();
    curve2.clear();
    Vector3D *tmp;
    tmp = new Vector3D(trilinearInterpolation(0.5,0.25,0.75));
    curve1.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.25,0.5,0.5));
    curve1.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.75,0.75,0.25));
    curve1.push_back(tmp);

    tmp = new Vector3D(trilinearInterpolation(0.25,0.25,0.25));
    curve2.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.75,0.5,0.5));
    curve2.push_back(tmp);
    tmp = new Vector3D(trilinearInterpolation(0.5,0.75,0.75));
    curve2.push_back(tmp);
}

void Tile::swapCurveMidPoints()
{
    int mid         = curve1.size()/2;
    Vector3D *temp  = curve1[mid];
    curve1[mid]     = curve2[mid];
    curve2[mid]     = temp;
}

void Tile::findCurveAndDirection(Vertex *end, Vertex *othEnd, bool &isCurve1, bool &isRev)
{
    if( knotType == PLUS1 || knotType == PLUS2 ) {
        if( (actVerts[0] == end && actVerts[1] == othEnd) ||
                (actVerts[1] == end && actVerts[0] == othEnd) ) {
            isCurve1    = true;
            isRev       = false;
        } else if( (actVerts[1] == end && actVerts[2] == othEnd) ||
                (actVerts[2] == end && actVerts[1] == othEnd) ) {
            isCurve1    = false;
            isRev       = true;
        }else if( (actVerts[2] == end && actVerts[3] == othEnd) ||
                (actVerts[3] == end && actVerts[2] == othEnd) ) {
            isCurve1    = true;
            isRev       = true;
        }else if( (actVerts[3] == end && actVerts[0] == othEnd) ||
                (actVerts[0] == end && actVerts[3] == othEnd) ) {
            isCurve1    = false;
            isRev       = false;
        }
    } else if( knotType == TWIST1 ) {
        if( (actVerts[0] == end && actVerts[1] == othEnd) ||
                (actVerts[1] == end && actVerts[0] == othEnd) ) {
            isCurve1 = true;
            isRev = false;
        } else if( (actVerts[1] == end && actVerts[2] == othEnd) ||
                (actVerts[2] == end && actVerts[1] == othEnd) ) {
            isCurve1 = false;
            isRev = false;
        }else if( (actVerts[2] == end && actVerts[3] == othEnd) ||
                (actVerts[3] == end && actVerts[2] == othEnd) ) {
            isCurve1 = false;
            isRev = true;
        }else if( (actVerts[3] == end && actVerts[0] == othEnd) ||
                (actVerts[0] == end && actVerts[3] == othEnd) ) {
            isCurve1 = true;
            isRev = true;
        }
    } else if( knotType == TWIST2 ) {
        if( (actVerts[0] == end && actVerts[1] == othEnd) ||
                (actVerts[1] == end && actVerts[0] == othEnd) ) {
            isCurve1 = true;
            isRev = false;
        } else if( (actVerts[1] == end && actVerts[2] == othEnd) ||
                (actVerts[2] == end && actVerts[1] == othEnd) ) {
            isCurve1 = true;
            isRev = true;
        }else if( (actVerts[2] == end && actVerts[3] == othEnd) ||
                (actVerts[3] == end && actVerts[2] == othEnd) ) {
            isCurve1 = false;
            isRev = true;
        }else if( (actVerts[3] == end && actVerts[0] == othEnd) ||
                (actVerts[0] == end && actVerts[3] == othEnd) ) {
            isCurve1 = false;
            isRev = false;
        }
    }
}





string Mesh::getOriginalFileName(string filename,string &path)
{
    vector<string> subStrs;
    char_separator<char> sep("\\");
    typedef tokenizer< boost::char_separator<char> > Tokenizer;
    Tokenizer tok(filename,sep);
    for(Tokenizer::iterator beg=tok.begin(); beg!=tok.end(); ++beg) subStrs.push_back(*beg);
    filename = subStrs[subStrs.size()-1];
    for( unsigned int k=0; k < subStrs.size()-1; ++k )
        path = path +"\\"+ subStrs[k];
    path	+= "\\";
    sep	= char_separator<char>(".");
    tok	= Tokenizer(filename,sep);
    subStrs.clear();
    for(Tokenizer::iterator beg=tok.begin(); beg!=tok.end(); ++beg) subStrs.push_back(*beg);
    return subStrs[0];
}

void Mesh::cacheFaceNormals()
{
    std::set<Face*> faces = this->GetFaces();
    for(std::set<Face*>::iterator fIter = faces.begin(); fIter!= faces.end(); fIter++)
    {
        float *temp = GetFaceNormal(*fIter);
        Vector3D *triadInput = new Vector3D(temp[0],temp[1],temp[2]);
        triadInput->normalize();
        fNormals_[*fIter] = triadInput;
    }
}

void Mesh::calculateVertexNormals()
{
    for (std::set<Vertex*>::iterator it = vertices_.begin(); it != vertices_.end(); ++it)
    {
        /* For each vertex find the averaged normal of the Faces */
        Vector3D *vertexNormal = new Vector3D(0.0f,0.0f,0.0f);
        Vertex *currVertex = (*it);
        std::vector<Edge*> currRotEdg = currVertex->GetRotation();
        unsigned int facesDone = currRotEdg.size();

        for(unsigned int iter=0; iter<facesDone; iter++)
        {
            float *temp = GetFaceNormal(face_map_[currVertex][currRotEdg[iter]->GetOtherEnd(currVertex)]);
            Vector3D vtmp(temp[0],temp[1],temp[2]);
            (*vertexNormal) += vtmp;
        }
        (*vertexNormal) /= facesDone;
        vertexNormal->normalize();
        vNormals_[currVertex] = vertexNormal;
    }
}

void Mesh::prepareTextureSets()
{
    texture_Bcorner.push_back('a');
    texture_Bcorner.push_back('b');
    texture_Bcorner.push_back('c');
    texture_Bcorner.push_back('d');

    texture_Ycorner.push_back('e');
    texture_Ycorner.push_back('f');
    texture_Ycorner.push_back('g');
    texture_Ycorner.push_back('h');

    compat_Bcorner.push_back('a');
    compat_Bcorner.push_back('c');
    compat_Bcorner.push_back('f');
    compat_Bcorner.push_back('h');

    compat_Ycorner.push_back('b');
    compat_Ycorner.push_back('d');
    compat_Ycorner.push_back('e');
    compat_Ycorner.push_back('g');
}

void Mesh::createTileTexCoordList()
{
    add2DCircularList(&_0aHead,'a',0.0,0.0);
    add2DCircularList(&_0aHead,'b',1.0,0.0);
    add2DCircularList(&_0aHead,'c',1.0,1.0);
    add2DCircularList(&_0aHead,'d',0.0,1.0);

    add2DCircularList(&_0bHead,'e',0.0,0.0);
    add2DCircularList(&_0bHead,'f',1.0,0.0);
    add2DCircularList(&_0bHead,'g',1.0,1.0);
    add2DCircularList(&_0bHead,'h',0.0,1.0);
    showDCircularList(_0aHead);
    showDCircularList(_0bHead);
}

bool Mesh::anyUnvisitedFaces(Face **retFacePntr,Tile **correspondingTile)
{
    for (std::set<Face*>::iterator fi = faces_.begin(); fi != faces_.end(); ++fi)
    {
        Tile *currTile = tile_map_[(*fi)];
        if( currTile->visited < 2 )
        {
            *retFacePntr = (*fi);
            *correspondingTile = currTile;
            return true;
        }
    }
    return false;
}

bool Mesh::LoadObjFile(const char *objFileName)
{
    /* clear object */
    if ( !(vertices_.empty()) ) {
        Clear();
    }
    /* save file name */
    fileName_ = objFileName;

    /* ------------------   load .obj file   ------------------ */
    FILE* inFile = fopen (objFileName, "r");
    if ( !inFile ) {
        ci::app::console() << "Can not open file: " << fileName_;
    }
    else {
        ci::app::console() << "Model file: " << fileName_;
    }
    /* 1st pass - read all the vertices */
    char buffer[512] = {0};
    float* coord;
    Vertex* newVertex;
    std::map<int, Vertex*> id_vertex;	/* vert ID starts from 1 */
    while ( fscanf( inFile, "%s", buffer) != EOF ) {
        if ( 'v' == buffer[0] ) {
            switch ( buffer[1] ) {
                case '\0':
                    coord = new float[3];
                    if ( fscanf(inFile, "%f %f %f", &(coord[0]), &(coord[1]), &(coord[2])) != 3 ) {
                        delete [] coord;
                        std::cout << "vertex " << vertices_.size() << " wrong" << std::endl;
                        return false;
                    }
                    else {
                        newVertex = AddVertex();
                        SetCoords(newVertex,coord);
                        id_vertex[vertices_.size()] = newVertex;
                        break;
                    }

                case 'n':
                case 't':
                    break;
            }
        }
    }
    ci::app::console() << "Number of vertices: " << vertices_.size() << std::endl;

    /* 2nd pass - add edges */
    numFacesObjFile_ = 0;
    int tempID, startID, endID;
    Edge* preEdge;
    Edge* newEdge;
    std::pair<int, int> edgeIDpair;	/* start <= end */
    std::pair<int, int> firstPair;	/* 1st edge pair */
    std::map<std::pair<int, int>, Edge*> vertex_id_edge;	/* vert ID starts from 1 */
    std::map<std::pair<int, int>, Edge*>::iterator it_edge;
    rewind(inFile);
    /* process each face */
    while ( fscanf(inFile, "%s", buffer) != EOF) {
        if ( 'f' == buffer[0] && '\0' == buffer[1] )
        {
            std::vector<int> fVertices;
            while ( fscanf(inFile, "%s", buffer) != EOF )
            {
                if ( 1 == sscanf(buffer, "%d", &tempID) )
                {
                    fVertices.push_back(tempID);
                }
                else
                {		/* finish one face */
                    numFacesObjFile_++;
                    if ( fVertices.size() < 2 )
                    {
                        std::cout << "Face only has " << fVertices.size() << " vertices" << std::endl;
                        return false;
                    }
                    /* 1st edge */
                    startID = fVertices[0];
                    endID = fVertices[1];
                    if ( startID > endID ) {
                        std::swap(startID, endID);
                    }
                    edgeIDpair.first = startID;
                    edgeIDpair.second = endID;
                    firstPair = edgeIDpair;
                    it_edge = vertex_id_edge.find(edgeIDpair);
                    if ( vertex_id_edge.end() == it_edge ) {	/* edge not created */
                        newEdge = ObjectStore::GetInstance()->CreateEdge(id_vertex[startID], id_vertex[endID]);
                        edges_.insert(newEdge);
                        vertex_id_edge[edgeIDpair] = newEdge;
                    }
                    else {
                        newEdge = it_edge->second;
                    }
                    preEdge = newEdge;
                    for (unsigned int i = 1; i < fVertices.size(); i++ )
                    {
                        startID = fVertices[i];
                        endID = fVertices[(i+1 == fVertices.size() ? 0 : i+1)];
                        if ( startID > endID ) {
                            std::swap(startID, endID);
                        }
                        edgeIDpair.first = startID;
                        edgeIDpair.second = endID;
                        it_edge = vertex_id_edge.find(edgeIDpair);
                        if ( vertex_id_edge.end() == it_edge) {		// edge not created
                            newEdge = ObjectStore::GetInstance()->CreateEdge(id_vertex[startID], id_vertex[endID]);
                            edges_.insert(newEdge);
                            vertex_id_edge[edgeIDpair] = newEdge;
                        }
                        else {
                            newEdge = it_edge->second;
                        }
                        /* add to rotation */
                        id_vertex[fVertices[i]]->InsertEdgeInRotation_load(newEdge, preEdge);
                        preEdge = newEdge;
                    }
                    // last & 1st edges in rotation
                    newEdge = vertex_id_edge[firstPair];
                    id_vertex[fVertices[0]]->InsertEdgeInRotation_load(newEdge, preEdge);

                    fVertices.clear();	// start a new face

                    if ( 'f' != buffer[0] )
                    {
                        break;
                    }
                }
            }
        }
    }
    fclose(inFile);
    ci::app::console() << "Number of edges: " << edges_.size() << std::endl;
    ci::app::console() << "Number of faces: " << numFacesObjFile_ << std::endl;
    ReComputeFaces();
    calculateVertexNormals();
    cacheFaceNormals();

    pumpMesh();
    ci::app::console() << "Face extrusion completed." << std::endl;
    traverseFaces_PLUSES();
    //traverseFaces_TWISTS();
    ci::app::console() << "Tile traversal completed." << std::endl;
    if(baseCrvType==CATMULLROMSPLINE) generateCtrlPnts4InterpolationSpline();
    generateWngTubes();
    return true;
}

bool Mesh::Clear()
{
    for (std::set<Vertex*>::iterator it = vertices_.begin();
            it != vertices_.end(); ++it) {
        (*it)->ClearRotations();
        ObjectStore::GetInstance()->DeleteVertex(*it);
    }
    vertices_.clear();

    for (std::set<Edge*>::iterator it = edges_.begin();
            it != edges_.end(); ++it) {
        ObjectStore::GetInstance()->DeleteEdge(*it);
    }
    edges_.clear();

    for (std::set<Face*>::iterator it = faces_.begin();
            it != faces_.end(); ++it) {
        ObjectStore::GetInstance()->DeleteFace(*it);
    }
    faces_.clear();
    face_map_.clear();
    fNormals_.clear();
    vNormals_.clear();
    tile_map_.clear();
    wngCrvs.clear();
    mWvngTubes.clear();
    numFacesObjFile_ = 0;
    vertex_treeIDMap_.clear();
    treeID_VertexSetMap_.clear();
    return true;
}

void Mesh::drawMesh(bool wireframe, bool lighting, bool drawBaseMesh, bool paintedMesh, bool tiledMesh, bool drawWeaving,
        GLint TURN1_OR_TOUCH, GLint TURN2_OR_CROSS, GLint DOUBLE_CROSS)
{
    if(paintedMesh)
    {
        glPushMatrix();
        for( std::set<Face*>::iterator fi = faces_.begin(); fi != faces_.end(); ++fi )
        {
            Tile* currT = tile_map_[*fi];
            // float *normal = GetFaceNormal(*fi);
            if( currT->tex == OA ) {
                glBindTexture(GL_TEXTURE_2D, TURN1_OR_TOUCH);
            }
            else if( currT->tex == OB ) {
                glBindTexture(GL_TEXTURE_2D, TURN2_OR_CROSS);
            }

            glBegin(GL_QUADS);
            // glNormal3fv(normal);
            for( vector<Vertex*>::iterator vIt = currT->actVerts.begin(); vIt != currT->actVerts.end(); ++vIt )
            {
                Vector3D *vnormal = vNormals_[*vIt];
                float *currVertex = coords_[*vIt];
                DCircularList *uvmap = currT->texCoords[*vIt];
                glNormal3f(vnormal->DX(), vnormal->DY(), vnormal->DZ());
                glTexCoord2f( uvmap->u, uvmap->v );
                glVertex3fv( currVertex );
            }
            glEnd();
            glBindTexture(GL_TEXTURE_2D, NULL);
        }
        glPopMatrix();
    }

    if(drawBaseMesh) {
        glPushMatrix();
        if(lighting) glEnable(GL_COLOR_MATERIAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
        for( std::set<Face*>::iterator fi = faces_.begin(); fi != faces_.end(); ++fi )
        {
            std::vector<Vertex*> cFVertices = (*fi)->GetVertices();
            glColor4f(1.0,1.0,1.0,1.0);
            glBegin(GL_POLYGON);
            glNormal3fv( GetFaceNormal(*fi) );
            for(unsigned int i=0;i<cFVertices.size();i++)
                glVertex3fv(GetVertexCoordinates(cFVertices[i]));
            glEnd();
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        if(lighting) glDisable(GL_COLOR_MATERIAL);
        glPopMatrix();
    }

    if(tiledMesh)
    {
        glPushMatrix();
        glDisable(GL_CULL_FACE);
        if(lighting) glEnable(GL_COLOR_MATERIAL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        for( std::set<Face*>::iterator fi = faces_.begin(); fi != faces_.end(); ++fi )
        {
            Vector3D *vtmp;
            Tile* currT = tile_map_[*fi];
            glBegin(GL_QUADS);
            float *normal = GetFaceNormal(*fi);
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);	// RED
            glNormal3fv(normal);
            for( Vec3DPtrIter it = currT->up.begin(); it != currT->up.end(); it++ )
            {
                vtmp = *it;
                glVertex3f( vtmp->DX(), vtmp->DY(), vtmp->DZ() );
            }

            glColor4f(0.0f, 1.0f, 0.0f, 1.0f);	// GREEN
            glNormal3f(-normal[0],-normal[1],-normal[2]);
            for( Vec3DPtrIter it = currT->down.begin(); it != currT->down.end(); it++ )
            {
                vtmp = *it;
                glVertex3f( vtmp->DX(), vtmp->DY(), vtmp->DZ() );
            }
            glEnd();

            glPointSize(4.0f);
            glBegin(GL_POINTS);
            vtmp = currT->curve1[1];
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);	// RED
            glVertex3f( vtmp->DX(), vtmp->DY(), vtmp->DZ() );
            vtmp = currT->curve2[1];
            glColor4f(0.0f, 1.0f, 0.0f, 1.0f);	// GREEN
            glVertex3f( vtmp->DX(), vtmp->DY(), vtmp->DZ() );
            glEnd();
            glPointSize(1.0f);
        }
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        if(lighting) glDisable(GL_COLOR_MATERIAL);
        glEnable(GL_CULL_FACE);
        glPopMatrix();
    }

    if(drawWeaving)
    {
        glPushMatrix();
        GLfloat mat_amb[]       = {0.05,0.05,0.05,1.0};
        GLfloat mat_diff[4];
        GLfloat mat_spec[]      = {1.0,1.0,1.0,1.0};
        GLfloat mat_shininess[] = {100.0};
        float r = 0.3f;
        float g = 0.3f;
        float b = 0.3f;
        for (std::vector<WvngTube*>::iterator tube = mWvngTubes.begin(); tube != mWvngTubes.end(); ++tube)
        {
            WvngTube* currTube = *tube;
            if( r < 1.0f )
                r += 0.2f;
            else if( g < 1.0f )
                g += 0.2f;
            else if( b < 1.0f )
                b += 0.2f;
            else {
                r = 0.3f;
                g = 0.3f;
                b = 0.3f;
            }
            mat_diff[0] = r; mat_diff[1] = g; mat_diff[2] = b; mat_diff[3] = 1.0;
            glPushMatrix();
            glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,mat_amb);
            glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,mat_diff);
            glMaterialfv(GL_FRONT,GL_SPECULAR,mat_spec);
            glMaterialfv(GL_FRONT,GL_SHININESS,mat_shininess);

            gluBeginSurface(theNurb);
            gluNurbsSurface(theNurb, currTube->uKnotCount, currTube->uKnots,
                    currTube->vKnotCount, currTube->vKnots,
                    currTube->uStride, currTube->vStride, currTube->ctlPoints,
                    currTube->uOrder, currTube->vOrder, GL_MAP2_VERTEX_3);
            gluEndSurface(theNurb);
            glPopMatrix();
        }
        /*glEnable(GL_COLOR_MATERIAL);
          glColor4f(1.0,1.0,1.0,1.0f);
          glPointSize(4.0);
          glBegin(GL_POINTS);
          for(vector<wngCrvCtrlPnts>::iterator it = wngCrvs.begin(); it != wngCrvs.end(); ++it )
          {
          for(Vec3DPtrIter v = (*it).myPnts.begin(); v != (*it).myPnts.end(); ++v )
          {
          Vector3D tmp = *(*v);
          glVertex3f(tmp.DX(),tmp.DY(),tmp.DZ());
          }
          break;
          }
          glEnd();
          glPointSize(1.0);
          glDisable(GL_COLOR_MATERIAL);
          */
        glPopMatrix();
    }
}

void Mesh::pumpMesh()
{
    /* Now move around faces and generate the positions */
    for (std::set<Face*>::iterator it = faces_.begin(); it != faces_.end(); ++it)
    {
        std::vector<Vertex*> faceVertices = (*it)->GetVertices();
        Tile *myTile = new Tile(faceVertices.size());

        for(unsigned int i=0; i<faceVertices.size(); ++i )
        {
            float *v = GetVertexCoordinates(faceVertices[i]);
            Vector3D base(v[0],v[1],v[2]);
            Vector3D normal = unitVector(*vNormals_[faceVertices[i]]);
            Vector3D *up = new Vector3D(base + TILE_THICKNESS*normal);      /* Move up */
            Vector3D *down = new Vector3D(base - TILE_THICKNESS*normal);    /* Move down */
            myTile->addUpAndDown(faceVertices[i],up,down);
        }
        myTile->generateCurvePoints();
        tile_map_[(*it)] = myTile;
    }
}

void Mesh::resetTexturePaintMarkers()
{
    set<Face*> faces = this->GetFaces();
    for(set<Face*>::iterator fIter = faces.begin(); fIter!= faces.end(); ++fIter) {
        tile_map_[*fIter]->marked	= -1;
        tile_map_[*fIter]->tex		= OA;
        tile_map_[*fIter]->texCoords.clear();
    }
    this->vertex_treeIDMap_.clear();
    this->treeID_VertexSetMap_.clear();
}

void Mesh::paintTexturesOnMesh()
{
    for( std::set<Vertex*>::iterator vi = vertices_.begin(); vi != vertices_.end(); ++vi )
    {
        char myColor			='0';
        std::vector<Edge*> edges= (*vi)->GetRotation();
        Vertex *end1			= *vi;
        Face *prevF				= 0;
        for(vector<Edge*>::iterator eIter = edges.begin(); eIter != edges.end(); ++eIter)
        {
            Vertex *end2= (*eIter)->GetOtherEnd(end1);
            Face* f		= face_map_[end1][end2];
            if( tile_map_[f]->marked != -1 ) {
                Tile* t		= tile_map_[f];
                char mark	= t->texCoords[end2]->name;
                vector<char>::iterator idx = find(texture_Bcorner.begin(), texture_Bcorner.end(), mark);
                myColor = texture_Bcorner.at(((idx-texture_Bcorner.begin())+1)%FACE_NUM_V);
                break;
            }
            prevF = f;
        }

        prevF = 0;
        if( myColor=='0' ) myColor = (*texture_Bcorner.begin());
        for( vector<Edge*>::iterator eIter = edges.begin(); eIter != edges.end(); ++eIter )
        {
            Vertex *end2= (*eIter)->GetOtherEnd(end1);
            Face* f		= face_map_[end1][end2];
            if( tile_map_[f]->marked == -1 )
            {
                Tile* t = tile_map_[f];
                vector<char>::iterator clrIndex = find(texture_Bcorner.begin(),texture_Bcorner.end(),myColor);
                int myClrPos					= clrIndex - texture_Bcorner.begin();

                int idx1,idx2;
                for(vector<Vertex*>::iterator tV = t->actVerts.begin(); tV != t->actVerts.end(); ++tV )
                {
                    if(end1 == *tV)
                        idx1 = tV - t->actVerts.begin();
                    else if(end2 == *tV)
                        idx2 = tV - t->actVerts.begin();
                }

                DCircularList *vTexPntr = rotateDCircularListFrwd(_0aHead,myClrPos);
                DCircularList *iter		= vTexPntr;
                if(idx1 < idx2)
                {
                    do {
                        t->texCoords[t->actVerts[idx1]]	= iter;
                        iter = iter->next;
                        if( !(++idx1 < 4) ) idx1 = 0;
                    }while(iter!=vTexPntr);
                } else {
                    do {
                        t->texCoords[t->actVerts[idx1]]	= iter;
                        iter = iter->prev;
                        if( !(--idx1 >= 0) ) idx1 = 3;
                    }while(iter!=vTexPntr);
                }
                t->marked	= t->marked + 1;
                t->tex		= OA;
            }
            prevF = f;
        }
    }
}

bool Mesh::findUnConnectedFace(Face **returnValue, Vertex **returnVertex)
{
    set<Face*> faces = this->GetFaces();
    for(set<Face*>::iterator fIter = faces.begin(); fIter != faces.end(); ++fIter)
    {
        if(tile_map_[*fIter]->marked==0 && tile_map_[*fIter]->tex==OA) {
            *returnValue = *fIter;
            vector<Vertex*> myVertices = (*returnValue)->GetVertices();
            Tile *currT = tile_map_[(*returnValue)];
            for( vector<Vertex*>::iterator vit = myVertices.begin(); vit != myVertices.end(); ++vit) {
                if( currT->texCoords[*vit]->name == 'b' || currT->texCoords[*vit]->name == 'd' ) {
                    *returnVertex = *vit;
                    break;
                }
            }
            return true;
        }
    }
    *returnValue = 0;
    return false;
}

void Mesh::incMarkerOnFacesIncident(Vertex *v)
{
    vector<Edge*> rot	= v->GetRotation();
    for(vector<Edge*>::iterator eIt = rot.begin(); eIt != rot.end(); ++eIt)
    {
        Tile *t = tile_map_[ face_map_[v][(*eIt)->GetOtherEnd(v)] ];
        t->marked = t->marked + 1;
    }
}

Vertex* Mesh::getNextPivotInFace(Face *f, Vertex *v, char &colorName, int &index1, int &index2)
{
    Tile *t = tile_map_[f];
    vector<char> matchBInOB(2);
    vector<char>::iterator newColorEnd = set_intersection(compat_Ycorner.begin(),compat_Ycorner.end(), texture_Bcorner.begin(), texture_Bcorner.end(), matchBInOB.begin());
    colorName = (*matchBInOB.begin());
    vector<Vertex*>::iterator index = find(t->actVerts.begin(), t->actVerts.end(),v);
    if(index != t->actVerts.end()) {
        index1 = index - t->actVerts.begin();
        index2 = (index1+1)%FACE_NUM_V;
        return t->actVerts.at( ( ( index1+2 )%FACE_NUM_V ) );
    } else {
        app::console()<<"Vertex not found, something fishy!"<<endl;
        return 0;
    }
}

void Mesh::connectBlobsPhase1()
{
    int curveID		= 0;
    bool firstRun	= true;
    Face *prevFace	= 0;
    Vertex *prevPivot= 0;
    Face *currFace	= 0;
    Vertex *pivot	= 0;
    Tile *currT		= 0;
    Colour BLUE(0.0,0.0,1.0);
    Colour CYAN(0.0,1.0,1.0);

    while(1)
    {
        if( prevPivot!=0 && (currFace==prevFace || currFace==0) ) {
            app::console()<<"Marking neighbour faces of the last vertex of the current curve"<<endl;
            incMarkerOnFacesIncident(prevPivot);
            app::console()<<"Done marking for this curve, proceeding to next curve if eligible."<<endl;
            prevPivot = 0;
        }
        if( !firstRun && currFace == prevFace && findUnConnectedFace(&currFace,&prevPivot) ) {
            ++curveID;
            app::console()<<"New tree started. I think so at least ;)"<<endl;
        }
        if(firstRun) {
            currFace	= (*this->GetFaces().begin());
            currT		= tile_map_[currFace];
            vector<Vertex*> myVertices = currFace->GetVertices();
            for( vector<Vertex*>::iterator vit = myVertices.begin(); vit != myVertices.end(); ++vit) {
                if( currT->texCoords[*vit]->name == 'b' || currT->texCoords[*vit]->name == 'd' ) {
                    prevPivot = *vit;
                    break;
                }
            }
            firstRun	= false;
        }

        if( currFace!=prevFace && currFace!=0 && prevPivot!= 0 )
        {
            char colorName;
            int idx1,idx2;
            currT				= tile_map_[currFace];
            incMarkerOnFacesIncident(prevPivot);
            idx1				= idx2 = -1;
            pivot				= getNextPivotInFace(currFace,prevPivot,colorName,idx1,idx2);

            if(! ( idx1>=0 && idx1<4 & idx2>=0 && idx2<4 ) ) app::console()<<"idx1,idx2 "<<idx1<<" "<<idx2<<endl;

            vector<char>::iterator findOut	= find(texture_Ycorner.begin(), texture_Ycorner.end(),colorName);
            DCircularList *vTexPntr			= rotateDCircularListFrwd(_0bHead, findOut-texture_Ycorner.begin());
            DCircularList *iter				= vTexPntr;
            if(idx1 < idx2) {
                do {
                    currT->texCoords[currT->actVerts[idx1]]	= iter;
                    iter = iter->next;
                    if( !(++idx1 < 4) ) idx1 = 0;
                }while(iter!=vTexPntr);
            } else {
                do {
                    currT->texCoords[currT->actVerts[idx1]]	= iter;
                    iter = iter->prev;
                    if( !(--idx1 >= 0) ) idx1 = 3;
                }while(iter!=vTexPntr);
            }
            currT->tex		= OB;
            vertex_treeIDMap_[prevPivot] = curveID;
            vertex_treeIDMap_[pivot] = curveID;
            treeID_VertexSetMap_[curveID].insert(prevPivot);
            treeID_VertexSetMap_[curveID].insert(pivot);
            prevPivot		= pivot;
            prevFace		= currFace;

            vector<Edge*> rot	= pivot->GetRotation();
            for(vector<Edge*>::iterator eIt = rot.begin(); eIt != rot.end(); ++eIt)
            {
                Face *temp = face_map_[pivot][(*eIt)->GetOtherEnd(pivot)];
                Tile *t = tile_map_[temp];
                if( t->marked == 0 && t->tex == OA && prevFace!=temp ) {
                    currFace = temp;
                    break;
                }
            }
        }else{
            app::console()<<"No next face assigned => Either all connected or we have disconnected trees."<<endl;
            app::console()<<"Curves extracted : "<<treeID_VertexSetMap_.size()<<endl;
            break;
        }
    }
}

void Mesh::connectBlobsPhase2()
{
    if( treeID_VertexSetMap_.size() > 1 )
    {
        vector<char> matchBInOB(2);
        vector<char>::iterator newColorEnd = set_intersection(compat_Ycorner.begin(),compat_Ycorner.end(), texture_Bcorner.begin(), texture_Bcorner.end(), matchBInOB.begin());
        vector<char>::iterator findOut	= find(texture_Ycorner.begin(), texture_Ycorner.end(),(*matchBInOB.begin()));
        DCircularList *vTexPntr			= rotateDCircularListFrwd(_0bHead, findOut-texture_Ycorner.begin());
        DCircularList *iter				= vTexPntr;

        set<Face*> faces = this->GetFaces();
        for(set<Face*>::iterator fIter = faces.begin(); fIter != faces.end(); ++fIter)
        {
            Tile *t		= tile_map_[*fIter];
            if( t->tex == OA && t->marked >= 2 )
            {
                Vertex *jumpIn = 0;
                vector<Vertex*> myVertices = (*fIter)->GetVertices();
                for( vector<Vertex*>::iterator vit = myVertices.begin(); vit != myVertices.end(); ++vit) {
                    if( t->texCoords[*vit]->name == 'b' || t->texCoords[*vit]->name == 'd' ) {
                        jumpIn = *vit;
                        break;
                    }
                }
                vector<Vertex*>::iterator index = find(t->actVerts.begin(), t->actVerts.end(),jumpIn);
                int idx1 = index - t->actVerts.begin();
                int idx2 = (idx1+2)%FACE_NUM_V;
                Vertex *jumpOtherEnd = t->actVerts.at(idx2);
                int v1CrvId = vertex_treeIDMap_[jumpIn];
                int v2CrvId = vertex_treeIDMap_[jumpOtherEnd];
                if( v1CrvId != v2CrvId )
                {
                    set<Vertex*> c2Vertices = treeID_VertexSetMap_[v2CrvId];
                    for(set<Vertex*>::iterator it = c2Vertices.begin(); it != c2Vertices.end(); ++it ) {
                        treeID_VertexSetMap_[v1CrvId].insert(*it);
                        vertex_treeIDMap_[*it] = v1CrvId;
                    }
                    map<int, std::set<Vertex*>>::iterator eraseCrv = treeID_VertexSetMap_.find(v2CrvId);
                    treeID_VertexSetMap_.erase(eraseCrv);
                    // Now assign tex coords according to new Tile OB for this face using idx1 & idx2
                    if(idx1 < idx2) {
                        do {
                            t->texCoords[t->actVerts[idx1]]	= iter;
                            iter = iter->next;
                            if( !(++idx1 < 4) ) idx1 = 0;
                        }while(iter!=vTexPntr);
                    } else {
                        do {
                            t->texCoords[t->actVerts[idx1]]	= iter;
                            iter = iter->prev;
                            if( !(--idx1 >= 0) ) idx1 = 3;
                        }while(iter!=vTexPntr);
                    }
                    t->tex		= OB;
                } else {
                    app::console()<<"Phase 2: OA Tile with same curve corners"<<endl;
                }
            }
        }
        app::console()<<"Number of curves after phase 2 : "<<treeID_VertexSetMap_.size()<<endl;
    } else
        app::console()<<"Only one curves extracted, proceed to phase 3 to process isolated vertices"<<endl;
}

void Mesh::connectBlobsPhase3()
{
    if( vertex_treeIDMap_.size() != vertices_.size() )
    {
        vector<char> matchBInOB(2);
        vector<char>::iterator newColorEnd = set_intersection(compat_Ycorner.begin(),compat_Ycorner.end(), texture_Bcorner.begin(), texture_Bcorner.end(), matchBInOB.begin());
        vector<char>::iterator findOut	= find(texture_Ycorner.begin(), texture_Ycorner.end(),(*matchBInOB.begin()));
        DCircularList *vTexPntr			= rotateDCircularListFrwd(_0bHead, findOut-texture_Ycorner.begin());
        DCircularList *iter				= vTexPntr;

        set<Face*> faces = this->GetFaces();
        for(set<Face*>::iterator fIter = faces.begin(); fIter != faces.end(); ++fIter)
        {
            Tile *t		= tile_map_[*fIter];
            if( t->tex == OA && t->marked == 1 ) {
                // Faces incident on isolated vertices will be still with markers set to 1
                Vertex *jumpIn = 0;
                vector<Vertex*> myVertices = (*fIter)->GetVertices();
                for( vector<Vertex*>::iterator vit = myVertices.begin(); vit != myVertices.end(); ++vit) {
                    if( t->texCoords[*vit]->name == 'b' || t->texCoords[*vit]->name == 'd' ) {
                        jumpIn = *vit;
                        break;
                    }
                }
                vector<Vertex*>::iterator index = find(t->actVerts.begin(), t->actVerts.end(),jumpIn);
                int idx1 = index - t->actVerts.begin();
                int idx2 = (idx1+2)%FACE_NUM_V;
                Vertex *jumpOtherEnd = t->actVerts.at(idx2);
                map<Vertex*,int>::iterator treeId = vertex_treeIDMap_.find(jumpIn);
                int v1CrvId = ( treeId != vertex_treeIDMap_.end() ? treeId->second : -1 );
                treeId = vertex_treeIDMap_.find(jumpOtherEnd);
                int v2CrvId = ( treeId != vertex_treeIDMap_.end() ? treeId->second : -1 );
                if( (v1CrvId!=-1 && v2CrvId==-1) || (v2CrvId!=-1 && v1CrvId==-1) ) {
                    if(v1CrvId!=-1 && v2CrvId==-1) {
                        vertex_treeIDMap_[jumpOtherEnd] = v1CrvId;
                        treeID_VertexSetMap_[v1CrvId].insert(jumpOtherEnd);
                        incMarkerOnFacesIncident(jumpOtherEnd);
                    } else {
                        vertex_treeIDMap_[jumpIn] = v2CrvId;
                        treeID_VertexSetMap_[v2CrvId].insert(jumpIn);
                        incMarkerOnFacesIncident(jumpIn);
                    }
                    // Now assign tex coords according to new Tile OB for this face using idx1 & idx2
                    if(idx1 < idx2) {
                        do {
                            t->texCoords[t->actVerts[idx1]]	= iter;
                            iter = iter->next;
                            if( !(++idx1 < 4) ) idx1 = 0;
                        }while(iter!=vTexPntr);
                    } else {
                        do {
                            t->texCoords[t->actVerts[idx1]]	= iter;
                            iter = iter->prev;
                            if( !(--idx1 >= 0) ) idx1 = 3;
                        }while(iter!=vTexPntr);
                    }
                    t->tex		= OB;
                }
            }
        }
    } else
        app::console()<<"No isolated vertices, hence phase 3 complete."<<endl;
}

void Mesh::writeWavefrontObj()
{
    // get output file names and paths
    string path;
    vector<string> extns; extns.push_back("*.obj,*.OBJ");
    getOriginalFileName(fileName_,path);
    fs::path newObjFilePath = app::getSaveFilePath(path,extns);
    if( !newObjFilePath.empty() ) {
        string mtlpath;
        string mtlfilename = getOriginalFileName(newObjFilePath.string(),mtlpath) + ".mtl";
        mtlpath += mtlfilename;
        string bcornerMtl = "BlueCorner";
        string ycornerMtl = "YellowCorner";

        //create index for vertices, normals and texture coordinates
        map<Vertex*,int> v2IndexMap;
        map<Vertex*,int> vNormalIndexMap;
        map<DCircularList*,int> texOAIndexMap, texOBIndexMap;
        int vIndex=1;
        for(set<Vertex*>::iterator vit = vertices_.begin(); vit != vertices_.end(); ++vit)
            v2IndexMap[*vit] = vIndex++;
        int vNormalIndex = 1;
        for(map<Vertex*, Vector3D*>::iterator vnit = vNormals_.begin(); vnit != vNormals_.end(); ++vnit)
            vNormalIndexMap[vnit->first] = vNormalIndex++;
        DCircularList *itOA = _0aHead->prev;
        int texOAIndex = 1;
        DCircularList *itOB = _0bHead->prev;
        int texOBIndex = 1;
        for(int k=0; k<4; ++k) {
            texOAIndexMap[itOA] = texOAIndex++; texOBIndexMap[itOB] = texOBIndex++;
            itOA = itOA->prev; itOB = itOB->prev;
        }

        //Now write the OBJ file and material files
        FILE *objHandle = fopen(newObjFilePath.string().c_str(),"w");
        fprintf(objHandle,"%s","# File format : Wavefront OBJ file \n");
        fprintf(objHandle,"%s","# Author : Pradeep Garigipati\n");
        fprintf(objHandle,"%s","# Date : 03/05/2010\n");
        fprintf(objHandle,"%s","# -----------------------------------\n\n");

        fprintf(objHandle,"%s","# Using the below material file \n");
        fprintf(objHandle,"mtllib %s\n\n",mtlfilename.c_str());

        fprintf(objHandle,"# Vertices set size = %d\n",v2IndexMap.size());
        for(map<Vertex*,int>::iterator v = v2IndexMap.begin(); v!= v2IndexMap.end(); ++v){
            float *coords = coords_[v->first];
            //app::console()<<"v "<<coords[0]<<" "<<coords[1]<<" "<<coords[2];
            fprintf(objHandle,"v %f %f %f\n",coords[0],coords[1],coords[2]);
        }
        fprintf(objHandle,"%s","\n\n");

        fprintf(objHandle,"# Vertex normals set size = %d\n",vNormalIndexMap.size());
        for(map<Vertex*,int>::iterator vn = vNormalIndexMap.begin(); vn!= vNormalIndexMap.end(); ++vn){
            Vector3D *normal = vNormals_[vn->first];
            //app::console()<<"v "<<normal->DX()<<" "<<normal->DY()<<" "<<normal->DZ()];
            fprintf(objHandle,"vn %f %f %f\n",normal->DX(),normal->DY(),normal->DZ());
        }
        fprintf(objHandle,"%s","\n\n");

        fprintf(objHandle,"# Texture coordinates set size = %d\n",texOAIndexMap.size());
        for(map<DCircularList*,int>::iterator texCrd = texOAIndexMap.begin(); texCrd!= texOAIndexMap.end(); ++texCrd){
            DCircularList *textureCoord = texCrd->first;
            fprintf(objHandle,"vt %f %f\n",textureCoord->u,textureCoord->v);
        }
        fprintf(objHandle,"%s","\n\n");

        fprintf(objHandle,"# Faces set size = %d\n",faces_.size());
        fprintf(objHandle,"usemtl %s\n", bcornerMtl.c_str() );
        for(set<Face*>::iterator f = faces_.begin(); f != faces_.end(); ++f) {
            Tile *t = tile_map_[*f];
            if(t->tex == OA) {
                string out="f ";
                vector<Vertex*> verts = (*f)->GetVertices();
                for(vector<Vertex*>::iterator vit = verts.begin(); vit != verts.end(); ++vit) {
                    stringstream ss;
                    string hold;
                    ss<<v2IndexMap[*vit]<<"/"<<texOAIndexMap[t->texCoords[*vit]]<<"/"<<vNormalIndexMap[*vit];
                    ss>>hold;
                    out = out + " " + hold;
                }
                fprintf(objHandle,"%s\n",out.c_str());
            }
        }
        fprintf(objHandle,"usemtl %s\n", ycornerMtl.c_str() );
        for(set<Face*>::iterator f = faces_.begin(); f != faces_.end(); ++f) {
            Tile *t = tile_map_[*f];
            if(t->tex == OB) {
                string out="f ";
                vector<Vertex*> verts = (*f)->GetVertices();
                for(vector<Vertex*>::iterator vit = verts.begin(); vit != verts.end(); ++vit) {
                    stringstream ss;
                    string hold;
                    ss<<v2IndexMap[*vit]<<"/"<< texOBIndexMap[t->texCoords[*vit]]<<"/"<<vNormalIndexMap[*vit];
                    ss>>hold;
                    out = out + " " + hold;
                }
                fprintf(objHandle,"%s\n",out.c_str());
            }
        }
        fprintf(objHandle,"%s","\n");
        fclose(objHandle);

        FILE *mtlHandle = fopen(mtlfilename.c_str(),"w");
        fprintf(mtlHandle,"%s","# File format : Material MTL file \n");
        fprintf(mtlHandle,"%s","# Author : Pradeep Garigipati\n");
        fprintf(mtlHandle,"%s","# Date : 03/05/2010\n\n");

        fprintf(mtlHandle,"newmtl %s\n",bcornerMtl.c_str());
        fprintf(mtlHandle,"Ka %f %f %f\n", 1.0, 1.0, 1.0);
        fprintf(mtlHandle,"Kd %f %f %f\n", 1.0, 1.0, 1.0);
        fprintf(mtlHandle,"Ks %f %f %f\n", 0.0, 0.0, 0.0);
        fprintf(mtlHandle,"d %f\n", 1.0);
        fprintf(mtlHandle,"illum %d\n", 2);
        fprintf(mtlHandle,"map_Ka -clamp on %s\n", "0a.png");
        fprintf(mtlHandle,"map_Kd -clamp on %s\n", "0a.png");
        fprintf(mtlHandle,"map_Ks -clamp on %s\n", "0a.png");

        fprintf(mtlHandle,"newmtl %s\n", ycornerMtl.c_str());
        fprintf(mtlHandle,"Ka %f %f %f\n", 1.0, 1.0, 1.0);
        fprintf(mtlHandle,"Kd %f %f %f\n", 1.0, 1.0, 1.0);
        fprintf(mtlHandle,"Ks %f %f %f\n", 0.0, 0.0, 0.0);
        fprintf(mtlHandle,"d %f\n", 1.0);
        fprintf(mtlHandle,"illum %d\n", 2);
        fprintf(mtlHandle,"map_Ka -clamp on %s\n", "0b.png");
        fprintf(mtlHandle,"map_Kd -clamp on %s\n", "0b.png");
        fprintf(mtlHandle,"map_Ks -clamp on %s\n", "0b.png");
        fclose(mtlHandle);
    }
}

void Mesh::traverseFaces_PLUSES()
{
    Face* currFace  = 0;
    Tile* currTile  = 0;

    while(anyUnvisitedFaces(&currFace,&currTile))
    {
        bool currCurve          = true;
        Edge* prevDirectionEdge = 0;
        KNOT prevFaceKnotType   = NOTHING;
        Face* prevFace          = 0;
        Tile* prevTile          = 0;
        Edge* directionEdge     = 0;
        wngCrvCtrlPnts currCrv;

        while(currCurve)
        {
            if(currTile->visited == 0)
            {
                Edge *edg       = 0;
                bool isCurve1,isReverse;
                Vertex *start   = 0;
                Vertex *end     = 0;
                // Pick the entry/random-entry edge and start,end vertices of the corresponding face
                if(directionEdge==0)
                    edg = (*(currFace->GetEdges().begin()));
                else
                    edg = directionEdge;
                start = edg->GetStart();
                end   = edg->GetEnd();
                if( face_map_[start][end] != currFace ) {
                    Vertex* temp= start;
                    start       = end;
                    end         = temp;
                }
                // Alternate the Tile patterns OR  generate TWIST points in even-face cycle
                if(prevFaceKnotType == currTile->knotType && (prevFaceKnotType == PLUS1 || prevFaceKnotType == PLUS2)) {
                    currTile->swapCurveMidPoints();
                    currTile->knotType = ( prevFaceKnotType == PLUS1 ? PLUS2 : PLUS1 );
                }else if(currTile->knotType == TWIST1)
                    currTile->generateTwist1Points();
                else if(currTile->knotType == TWIST2)
                    currTile->generateTwist2Points();
                // find the curve of entry based on the entry edge
                currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                // Push the corresponding curve points
                if(isCurve1) {
                    if(isReverse) {
                        /*for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                          currCrv.myPnts.push_back(*iter);*/
                        for( int crv=currTile->curve1.size()-2; crv>0; crv--)
                            currCrv.myPnts.push_back(currTile->curve1[crv]);
                        currTile->isUsedCrv1Rev = YES;
                        Vector3D *derv = new Vector3D((*(*currTile->curve1.begin()))-(*(*currTile->curve1.rbegin())));
                        derv->normalize();
                        currCrv.derivatives.push_back(derv);
                    } else {
                        /*for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                          currCrv.myPnts.push_back(*iter);*/
                        for( int crv=1; crv<currTile->curve1.size()-1; crv++)
                            currCrv.myPnts.push_back(currTile->curve1[crv]);
                        currTile->isUsedCrv1Rev = NO;
                        Vector3D *derv = new Vector3D((*(*currTile->curve1.rbegin()))-(*(*currTile->curve1.begin())));
                        derv->normalize();
                        currCrv.derivatives.push_back(derv);
                    }
                    currTile->isCrv1Used = YES;
                } else {
                    if(isReverse) {
                        /*for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                          currCrv.myPnts.push_back(*iter);*/
                        for( int crv=currTile->curve2.size()-2; crv>0; crv--)
                            currCrv.myPnts.push_back(currTile->curve2[crv]);
                        currTile->isUsedCrv2Rev = YES;
                        Vector3D *derv = new Vector3D((*(*currTile->curve2.begin()))-(*(*currTile->curve2.rbegin())));
                        derv->normalize();
                        currCrv.derivatives.push_back(derv);
                    } else {
                        /*for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                          currCrv.myPnts.push_back(*iter);*/
                        for( int crv=1; crv<currTile->curve2.size()-1; crv++)
                            currCrv.myPnts.push_back(currTile->curve2[crv]);
                        currTile->isUsedCrv2Rev = NO;
                        Vector3D *derv = new Vector3D((*(*currTile->curve2.rbegin()))-(*(*currTile->curve2.begin())));
                        derv->normalize();
                        currCrv.derivatives.push_back(derv);
                    }
                    currTile->isCrv2Used = YES;
                }
                // store face normal to help generate thread
                currCrv.triadNorms.push_back(fNormals_[currFace]);
                currTile->visited++;
                // extract the direction edge based on curve used
                if(currTile->knotType == TWIST1) {
                    if(isCurve1)
                        directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                    else
                        directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                } else if(currTile->knotType == TWIST2) {
                    if(isCurve1)
                        directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                    else
                        directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                } else {
                    Edge* adjEdge = end->GetNextEdgeInRotation(edg);
                    Vertex *adjEdgOtherEnd = adjEdge->GetOtherEnd(end);
                    directionEdge = adjEdgOtherEnd->GetNextEdgeInRotation(adjEdge);
                }
                // Store the current face details as prev face details for next face
                prevDirectionEdge = edg;
                prevFaceKnotType = currTile->knotType;
                prevFace = currFace;
                prevTile = currTile;
                // Update current Face and corresponding Tile variables
                start = directionEdge->GetStart();
                end = directionEdge->GetEnd();
                if( face_map_[start][end] == currFace ) {
                    Vertex* temp= start;
                    start       = end;
                    end         = temp;
                }
                currFace        = face_map_[start][end];
                currTile        = tile_map_[currFace];
            }
            else if( currTile->visited == 1 )
            {
                Edge *edg       = 0;
                Vertex *start   = 0;
                Vertex *end     = 0;
                bool isCurve1,isReverse;

                if(directionEdge==0) {
                    // Loop over the edges to find an edge which emits an unvisited curve
                    std::vector<Edge*> currFaceEdges = currFace->GetEdges();
                    for( std::vector<Edge*>::iterator e = currFaceEdges.begin(); e!=currFaceEdges.end(); e++ )
                    {
                        Edge* tmp   = *e;
                        start       = tmp->GetStart();
                        end         = tmp->GetEnd();
                        if( face_map_[start][end] != currFace ) {
                            Vertex* temp= start;
                            start       = end;
                            end         = temp;
                        }
                        currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                        if((currTile->isCrv1Used == YES && isCurve1) || (currTile->isCrv2Used == YES && !isCurve1))
                            continue;
                        else {
                            edg = tmp;
                            break;
                        }
                    }
                }
                else {// Use the common edge from prev step
                    edg = directionEdge;
                    start = edg->GetStart();
                    end = edg->GetEnd();
                    if( face_map_[start][end] != currFace ) {
                        Vertex* temp = start;
                        start = end;
                        end = temp;
                    }
                    currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                }

                if( (prevFaceKnotType == PLUS1 || prevFaceKnotType == PLUS2) &&
                        (currTile->knotType == PLUS1 || currTile->knotType == PLUS2) )
                {
                    if((currTile->isCrv1Used == YES && isCurve1) ||
                            (currTile->isCrv2Used == YES && !isCurve1))
                    {
                        if( prevFaceKnotType==currTile->knotType )
                        {
                            // Remove already pushed curve vertices
                            for(unsigned int rem=0; rem<3; rem++)
                                currCrv.myPnts.pop_back();
                            // Change prev face KNOT type; choose TWIST1
                            prevTile->knotType = TWIST1;
                            prevTile->visited--;
                            // Now reset the state variables to prev state and continue with new KNOT type.
                            // Don't change the visited of the currTile
                            prevFaceKnotType = TWIST1;
                            currFace = prevFace;
                            currTile = prevTile;
                            directionEdge = prevDirectionEdge;
                        } else {
                            // This is one special case where we should close curve
                            // since we entered the face along the edge we started the curve
                            // for(int i=0;i<1;i++) {
                            currCrv.myPnts.push_back(currCrv.myPnts[0]);
                            currCrv.derivatives.push_back(currCrv.derivatives[0]);
                            currCrv.triadNorms.push_back(currCrv.triadNorms[0]);
                            // }
                            currCurve = false;
                            wngCrvs.push_back(currCrv);
                        }
                    }
                    else
                    {
                        // If curve we are gng to use is not used yet; just proceed in normal fashion
                        // Add points and compute next direction and update prev* variables
                        if(isCurve1) {
                            if(isReverse) {
                                /*for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                                  currCrv.myPnts.push_back(*iter);*/
                                for( int crv=currTile->curve1.size()-2; crv>0; crv--)
                                    currCrv.myPnts.push_back(currTile->curve1[crv]);
                                Vector3D *derv = new Vector3D((*(*currTile->curve1.begin()))-(*(*currTile->curve1.rbegin())));
                                derv->normalize();
                                currCrv.derivatives.push_back(derv);
                            } else {
                                /*for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                                  currCrv.myPnts.push_back(*iter);*/
                                for( int crv=1; crv<currTile->curve1.size()-1; crv++)
                                    currCrv.myPnts.push_back(currTile->curve1[crv]);
                                Vector3D *derv = new Vector3D((*(*currTile->curve1.rbegin()))-(*(*currTile->curve1.begin())));
                                derv->normalize();
                                currCrv.derivatives.push_back(derv);
                            }
                            currTile->isCrv1Used = YES;
                        } else {
                            if(isReverse) {
                                /*for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                                  currCrv.myPnts.push_back(*iter);*/
                                for( int crv=currTile->curve2.size()-2; crv>0; crv--)
                                    currCrv.myPnts.push_back(currTile->curve2[crv]);
                                Vector3D *derv = new Vector3D((*(*currTile->curve2.begin()))-(*(*currTile->curve2.rbegin())));
                                derv->normalize();
                                currCrv.derivatives.push_back(derv);
                            } else {
                                /*for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                                  currCrv.myPnts.push_back(*iter);*/
                                for( int crv=1; crv<currTile->curve2.size()-1; crv++)
                                    currCrv.myPnts.push_back(currTile->curve2[crv]);
                                Vector3D *derv = new Vector3D((*(*currTile->curve2.rbegin()))-(*(*currTile->curve2.begin())));
                                derv->normalize();
                                currCrv.derivatives.push_back(derv);
                            }
                            currTile->isCrv2Used = YES;
                        }
                        currCrv.triadNorms.push_back(fNormals_[currFace]);
                        currTile->visited++;
                        // Store the current face details as prev face details for next face
                        prevDirectionEdge = edg;
                        prevFaceKnotType = currTile->knotType;
                        prevFace = currFace;
                        prevTile = currTile;
                        // compute next direction
                        Edge* adjEdge = end->GetNextEdgeInRotation(edg);
                        Vertex *adjEdgOtherEnd = adjEdge->GetOtherEnd(end);
                        directionEdge = adjEdgOtherEnd->GetNextEdgeInRotation(adjEdge);
                        // Update current Face and corresponding Tile variables
                        start = directionEdge->GetStart();
                        end = directionEdge->GetEnd();
                        if( face_map_[start][end] == currFace ) {
                            Vertex* temp = start;
                            start = end;
                            end = temp;
                        }
                        currFace = face_map_[start][end];
                        currTile = tile_map_[currFace];
                    }
                }
                else
                {
                    currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                    if(currTile->knotType == TWIST1)
                        currTile->generateTwist1Points();
                    else if(currTile->knotType == TWIST2)
                        currTile->generateTwist2Points();

                    if(isCurve1) {
                        if(isReverse) {
                            /*for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                              currCrv.myPnts.push_back(*iter);*/
                            for( int crv=currTile->curve1.size()-2; crv>0; crv--)
                                currCrv.myPnts.push_back(currTile->curve1[crv]);
                            Vector3D *derv = new Vector3D((*(*currTile->curve1.begin()))-(*(*currTile->curve1.rbegin())));
                            derv->normalize();
                            currCrv.derivatives.push_back(derv);
                        } else {
                            /*for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                              currCrv.myPnts.push_back(*iter);*/
                            for( int crv=1; crv<currTile->curve1.size()-1; crv++)
                                currCrv.myPnts.push_back(currTile->curve1[crv]);
                            Vector3D *derv = new Vector3D((*(*currTile->curve1.rbegin()))-(*(*currTile->curve1.begin())));
                            derv->normalize();
                            currCrv.derivatives.push_back(derv);
                        }
                        currTile->isCrv1Used = YES;
                    } else {
                        if(isReverse) {
                            /*for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                              currCrv.myPnts.push_back(*iter);*/
                            for( int crv=currTile->curve2.size()-2; crv>0; crv--)
                                currCrv.myPnts.push_back(currTile->curve2[crv]);
                            Vector3D *derv = new Vector3D((*(*currTile->curve2.begin()))-(*(*currTile->curve2.rbegin())));
                            derv->normalize();
                            currCrv.derivatives.push_back(derv);
                        } else {
                            /*for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                              currCrv.myPnts.push_back(*iter);*/
                            for( int crv=1; crv<currTile->curve2.size()-1; crv++)
                                currCrv.myPnts.push_back(currTile->curve2[crv]);
                            Vector3D *derv = new Vector3D((*(*currTile->curve2.rbegin()))-(*(*currTile->curve2.begin())));
                            derv->normalize();
                            currCrv.derivatives.push_back(derv);
                        }
                        currTile->isCrv2Used = YES;
                    }
                    currCrv.triadNorms.push_back(fNormals_[currFace]);
                    currTile->visited++;
                    // Store the current face details as prev face details for next face
                    prevDirectionEdge = edg;
                    prevFaceKnotType = currTile->knotType;
                    prevFace = currFace;
                    prevTile = currTile;

                    if(currTile->knotType == TWIST1) {
                        if(isCurve1)
                            directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                        else
                            directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                    } else if(currTile->knotType == TWIST2) {
                        if(isCurve1)
                            directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                        else
                            directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                    } else {
                        Edge* adjEdge = end->GetNextEdgeInRotation(edg);
                        Vertex *adjEdgOtherEnd = adjEdge->GetOtherEnd(end);
                        directionEdge = adjEdgOtherEnd->GetNextEdgeInRotation(adjEdge);
                    }
                    // Update current Face and corresponding Tile variables
                    start = directionEdge->GetStart();
                    end = directionEdge->GetEnd();
                    if( face_map_[start][end] == currFace ) {
                        Vertex* temp = start;
                        start = end;
                        end = temp;
                    }
                    currFace = face_map_[start][end];
                    currTile = tile_map_[currFace];
                }
            }
            else if( currTile->visited == 2 )
            {
                // for(int i=0;i<1;i++) {
                currCrv.myPnts.push_back(currCrv.myPnts[0]);
                currCrv.derivatives.push_back(currCrv.derivatives[0]);
                currCrv.triadNorms.push_back(currCrv.triadNorms[0]);
                // }
                currCurve = false;
                wngCrvs.push_back(currCrv);
            }
        }
    }
}

void Mesh::traverseFaces_TWISTS()
{
    Face* currFace=0;
    Tile* currTile=0;

    while(anyUnvisitedFaces(&currFace,&currTile))
    {
        bool currCurve=true;
        Edge* prevDirectionEdge=0;
        KNOT prevFaceKnotType = NOTHING;
        Face* prevFace=0;
        Tile* prevTile=0;
        Edge* directionEdge = 0;
        wngCrvCtrlPnts currCrv;

        while(currCurve)
        {
            if(currTile->visited == 0)
            {
                Edge *edg;
                if(directionEdge==0)
                    edg = (*(currFace->GetEdges().begin())); // Pick a random edge
                else
                    edg = directionEdge;   // Use the common edge identified in previous iteration
                Vertex *start = edg->GetStart();
                Vertex *end = edg->GetEnd();
                if( face_map_[start][end] != currFace ) {
                    Vertex* temp = start;
                    start = end;
                    end = temp;
                }
                bool isCurve1;
                bool isReverse;
                if(prevFaceKnotType == currTile->knotType) {
                    if(currTile->knotType == PLUS1 ) {
                        currTile->swapCurveMidPoints();
                        currTile->knotType = PLUS2;
                    } else if(currTile->knotType == TWIST1) {
                        currTile->knotType = TWIST2;
                        currTile->generateTwist2Points();
                    } else if(currTile->knotType == TWIST2) {
                        currTile->knotType = TWIST1;
                        currTile->generateTwist1Points();
                    }
                } else if(currTile->knotType == PLUS1)
                    currTile->generateCurvePoints();
                else if(currTile->knotType == PLUS2)
                    currTile->swapCurveMidPoints();
                else if(currTile->knotType == TWIST1)
                    currTile->generateTwist1Points();
                else if(currTile->knotType == TWIST2)
                    currTile->generateTwist2Points();

                currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                if(isCurve1) {
                    if(isReverse) {
                        for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    } else {
                        for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    }
                    currTile->isCrv1Used = YES;
                } else {
                    if(isReverse) {
                        for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    } else {
                        for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    }
                    currTile->isCrv2Used = YES;
                }
                currCrv.triadNorms.push_back(fNormals_[currFace]);
                currTile->visited++;

                if(currTile->knotType == TWIST1) {
                    if(isCurve1)
                        directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                    else
                        directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                } else if(currTile->knotType == TWIST2) {
                    if(isCurve1)
                        directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                    else
                        directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                } else {
                    Edge* adjEdge = end->GetNextEdgeInRotation(edg);
                    Vertex *adjEdgOtherEnd = adjEdge->GetOtherEnd(end);
                    directionEdge = adjEdgOtherEnd->GetNextEdgeInRotation(adjEdge);
                }
                // Store the current face details as prev face details for next face
                prevDirectionEdge = edg;
                prevFaceKnotType = currTile->knotType;
                prevFace = currFace;
                prevTile = currTile;
                // Update current Face and corresponding Tile variables
                start = directionEdge->GetStart();
                end = directionEdge->GetEnd();
                if( face_map_[start][end] == currFace ) {
                    Vertex* temp = start;
                    start = end;
                    end = temp;
                }
                currFace = face_map_[start][end];
                currTile = tile_map_[currFace];
            }
            else if( currTile->visited == 1 )
            {
                Edge *edg=0;
                Vertex *start=0;
                Vertex *end=0;
                bool isCurve1;
                bool isReverse;

                if(directionEdge==0) {
                    std::vector<Edge*> currFaceEdges = currFace->GetEdges();
                    // Loop over the edges until you find an edge from which an unvisited curve is coming out
                    for( std::vector<Edge*>::iterator e = currFaceEdges.begin(); e!=currFaceEdges.end(); e++ )
                    {
                        Edge* tmp = *e;
                        start = tmp->GetStart();
                        end = tmp->GetEnd();
                        if( face_map_[start][end] != currFace ) {
                            Vertex* temp = start;
                            start = end;
                            end = temp;
                        }
                        currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                        if(!((currTile->isCrv1Used == YES && isCurve1) || (currTile->isCrv2Used == YES && !isCurve1)))
                        {
                            edg = tmp;
                            break;
                        }
                    }
                }
                else {
                    edg = directionEdge;   // Use the common edge identified in previous iteration
                    start = edg->GetStart();
                    end = edg->GetEnd();
                    if( face_map_[start][end] != currFace ) {
                        Vertex* temp = start;
                        start = end;
                        end = temp;
                    }
                    currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                }


                if( ( (prevFaceKnotType == PLUS1 || prevFaceKnotType == PLUS2) && (currTile->knotType == PLUS1 || currTile->knotType == PLUS2) ) ||
                        ( (prevFaceKnotType == TWIST1 || prevFaceKnotType == TWIST2) && (currTile->knotType == TWIST1 || currTile->knotType == TWIST2) ) )
                {
                    if((currTile->isCrv1Used == YES && isCurve1) || (currTile->isCrv2Used == YES && !isCurve1))
                    {
                        if( prevFaceKnotType==currTile->knotType )
                        {
                            // Remove already pushed curve vertices
                            for(unsigned int rem=0; rem<3; rem++)
                                currCrv.myPnts.pop_back();
                            // Change prev face KNOT type; choose TWIST1
                            prevTile->knotType = PLUS1;
                            prevTile->visited--;
                            // Now reset the state variables to prev state and continue with new KNOT type.
                            // Don't change the visited of the currTile
                            prevFaceKnotType = PLUS1;
                            currFace = prevFace;
                            currTile = prevTile;
                            directionEdge = prevDirectionEdge;
                        } else {
                            // This is one special case where we should close curve
                            // since we entered the face along the edge we started the curve
                            if(isCurve1) {
                                if(isReverse) {
                                    for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                                        currCrv.myPnts.push_back(*iter);
                                } else {
                                    for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                                        currCrv.myPnts.push_back(*iter);
                                }
                            } else {
                                if(isReverse) {
                                    for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                                        currCrv.myPnts.push_back(*iter);
                                } else {
                                    for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                                        currCrv.myPnts.push_back(*iter);
                                }
                            }
                            currCrv.triadNorms.push_back(fNormals_[currFace]);
                            currCurve = false;
                            wngCrvs.push_back(currCrv);
                        }
                    }
                    else
                    {
                        // If curve we are gng to use is not used yet; just proceed in normal fashion
                        // Add points and compute next direction and update prev* variables
                        if(isCurve1) {
                            if(isReverse) {
                                for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                                    currCrv.myPnts.push_back(*iter);
                            } else {
                                for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                                    currCrv.myPnts.push_back(*iter);
                            }
                            currTile->isCrv1Used = YES;
                        } else {
                            if(isReverse) {
                                for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                                    currCrv.myPnts.push_back(*iter);
                            } else {
                                for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                                    currCrv.myPnts.push_back(*iter);
                            }
                            currTile->isCrv2Used = YES;
                        }
                        currCrv.triadNorms.push_back(fNormals_[currFace]);
                        currTile->visited++;
                        // Store the current face details as prev face details for next face
                        prevDirectionEdge = edg;
                        prevFaceKnotType = currTile->knotType;
                        prevFace = currFace;
                        prevTile = currTile;
                        // compute next direction
                        if(currTile->knotType == TWIST1) {
                            if(isCurve1)
                                directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                            else
                                directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                        } else if(currTile->knotType == TWIST2) {
                            if(isCurve1)
                                directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                            else
                                directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                        } else {
                            Edge* adjEdge = end->GetNextEdgeInRotation(edg);
                            Vertex *adjEdgOtherEnd = adjEdge->GetOtherEnd(end);
                            directionEdge = adjEdgOtherEnd->GetNextEdgeInRotation(adjEdge);
                        }
                        // Update current Face and corresponding Tile variables
                        start = directionEdge->GetStart();
                        end = directionEdge->GetEnd();
                        if( face_map_[start][end] == currFace ) {
                            Vertex* temp = start;
                            start = end;
                            end = temp;
                        }
                        currFace = face_map_[start][end];
                        currTile = tile_map_[currFace];
                    }
                }
                else
                {
                    currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                    /*if(currTile->knotType == TWIST1)
                      currTile->generateTwist1Points();
                      else if(currTile->knotType == TWIST2)
                      currTile->generateTwist2Points();*/

                    if(isCurve1) {
                        if(isReverse) {
                            for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                                currCrv.myPnts.push_back(*iter);
                        } else {
                            for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                                currCrv.myPnts.push_back(*iter);
                        }
                        currTile->isCrv1Used = YES;
                    } else {
                        if(isReverse) {
                            for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                                currCrv.myPnts.push_back(*iter);
                        } else {
                            for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                                currCrv.myPnts.push_back(*iter);
                        }
                        currTile->isCrv2Used = YES;
                    }
                    currCrv.triadNorms.push_back(fNormals_[currFace]);
                    currTile->visited++;
                    // Store the current face details as prev face details for next face
                    prevDirectionEdge = edg;
                    prevFaceKnotType = currTile->knotType;
                    prevFace = currFace;
                    prevTile = currTile;

                    if(currTile->knotType == TWIST1) {
                        if(isCurve1)
                            directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                        else
                            directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                    } else if(currTile->knotType == TWIST2) {
                        if(isCurve1)
                            directionEdge = (isReverse==true ? start->GetPreviousEdgeInRotation(edg) : end->GetNextEdgeInRotation(edg));
                        else
                            directionEdge = (isReverse==true ? end->GetNextEdgeInRotation(edg) : start->GetPreviousEdgeInRotation(edg));
                    } else {
                        Edge* adjEdge = end->GetNextEdgeInRotation(edg);
                        Vertex *adjEdgOtherEnd = adjEdge->GetOtherEnd(end);
                        directionEdge = adjEdgOtherEnd->GetNextEdgeInRotation(adjEdge);
                    }
                    // Update current Face and corresponding Tile variables
                    start = directionEdge->GetStart();
                    end = directionEdge->GetEnd();
                    if( face_map_[start][end] == currFace ) {
                        Vertex* temp = start;
                        start = end;
                        end = temp;
                    }
                    currFace = face_map_[start][end];
                    currTile = tile_map_[currFace];
                }
            }
            else if( currTile->visited == 2 )
            {
                Edge *edg = directionEdge;   // Use the common edge identified in previous iteration
                Vertex *start = edg->GetStart();
                Vertex *end = edg->GetEnd();
                if( face_map_[start][end] != currFace ) {
                    Vertex* temp = start;
                    start = end;
                    end = temp;
                }

                bool isCurve1;
                bool isReverse;
                currTile->findCurveAndDirection(start,end,isCurve1,isReverse);
                if(isCurve1) {
                    if(isReverse) {
                        for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve1.rbegin(); iter != currTile->curve1.rend(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    } else {
                        for( std::vector<Vector3D*>::iterator iter = currTile->curve1.begin(); iter != currTile->curve1.end(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    }
                } else {
                    if(isReverse) {
                        for( std::vector<Vector3D*>::reverse_iterator iter = currTile->curve2.rbegin(); iter != currTile->curve2.rend(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    } else {
                        for( std::vector<Vector3D*>::iterator iter = currTile->curve2.begin(); iter != currTile->curve2.end(); iter++ )
                            currCrv.myPnts.push_back(*iter);
                    }
                }
                currCrv.triadNorms.push_back(fNormals_[currFace]);
                currCurve = false;
                wngCrvs.push_back(currCrv);
            }
        }
    }
}


void Mesh::generateCtrlPnts4InterpolationSpline()
{
    for (std::vector<wngCrvCtrlPnts>::iterator crv = wngCrvs.begin(); crv != wngCrvs.end(); ++crv)
    {
        Vec3DPtrIter vBegin			= (*crv).myPnts.begin();
        Vec3DPtrIter vEnd			= (*crv).myPnts.end();
        Vec3DPtrIter dBegin			= (*crv).derivatives.begin();
        Vec3DPtrIter dEnd			= (*crv).derivatives.end();
        Vec3DPtrIter nBegin			= (*crv).triadNorms.begin();
        Vec3DPtrIter nEnd			= (*crv).triadNorms.end();
        Vector3D *vFinish			= *((*crv).myPnts.rbegin()+1);
        Vec3DPtrIter dIter			= dBegin;
        Vec3DPtrIter nIter			= nBegin;

        for( Vec3DPtrIter c = vBegin; c+1 != vEnd; ++c, ++dIter, ++nIter )
        {
            Vector3D P1, P2, P3, P4, b1, b2, derv, norm, binorm;
            Vector3D n1 = *(*nIter);
            Vector3D n2 = ( c+2 == vEnd ? *(*(nBegin+1)) : *(*(nIter+1)) );

            if( c == vBegin ) {
                P1.set( vFinish->DX(), vFinish->DY(), vFinish->DZ() );
                P2.set( (*c)->DX(), (*c)->DY(), (*c)->DZ() );
                P3.set( (*(c+1))->DX(), (*(c+1))->DY(), (*(c+1))->DZ() );
                P4.set( (*(c+2))->DX(), (*(c+2))->DY(), (*(c+2))->DZ() );
            } else if( c+2 == vEnd ) {
                P1.set( (*(c-3))->DX(), (*(c-3))->DY(), (*(c-3))->DZ() );
                P2.set( (*c)->DX(), (*c)->DY(), (*c)->DZ() );
                P3.set( (*(c+1))->DX(), (*(c+1))->DY(), (*(c+1))->DZ() );
                P4.set( (*(vBegin+3))->DX(), (*(vBegin+3))->DY(), (*(vBegin+3))->DZ() );
            } else {
                P1.set( (*(c-3))->DX(), (*(c-3))->DY(), (*(c-3))->DZ() );
                P2.set( (*c)->DX(), (*c)->DY(), (*c)->DZ() );
                P3.set( (*(c+1))->DX(), (*(c+1))->DY(), (*(c+1))->DZ() );
                P4.set( (*(c+2))->DX(), (*(c+2))->DY(), (*(c+2))->DZ() );
            }
            b1 = P2 + (P3 - P1) / 6.0f;
            b2 = P3 + (P2 - P4) / 6.0f;

            c++; dIter++; nIter++;
            c = (*crv).myPnts.insert(c,new Vector3D(b1));
            derv = b2 - P2; derv.normalize();
            binorm = cross(derv,n1); binorm.normalize();
            norm = cross(binorm,derv); norm.normalize();
            dIter = (*crv).derivatives.insert(dIter,new Vector3D(derv));
            nIter = (*crv).triadNorms.insert(nIter,new Vector3D(norm));

            c++; dIter++; nIter++;
            c = (*crv).myPnts.insert(c,new Vector3D(b2));
            derv = P3 - b1; derv.normalize();
            binorm = cross(derv,n2); binorm.normalize();
            norm = cross(binorm,derv); norm.normalize();
            dIter = (*crv).derivatives.insert(dIter,new Vector3D(derv));
            nIter = (*crv).triadNorms.insert(nIter,new Vector3D(norm));

            /* Begin and End iterators are no longer valid, hence get updated ones */
            vBegin	= (*crv).myPnts.begin();
            vEnd	= (*crv).myPnts.end();
            nBegin	= (*crv).triadNorms.begin();
        }
    }
}


void Mesh::generateKnotVectors(GLfloat *uKnots, GLint uKnotCount, GLfloat *vKnots, GLint vKnotCount)
{
    GLfloat step    = 1.0;
    GLint ktIter    = 0;
    uKnots[ktIter++]= 0.0f;
    for(GLint k=ktIter; k<uKnotCount; k++ )
        uKnots[k] = uKnots[k-1] + step;
    ktIter          = 0;
    vKnots[ktIter++]= 0.0f;
    for(GLint k=ktIter; k<vKnotCount; k++ )
        vKnots[k] = vKnots[k-1] + step;
}

void Mesh::generateTubeCtrlPnts(const wngCrvCtrlPnts &wvngCrv, float *ctlpoints)
{
    int i;
    int triadIndex = 0;
    int firstRunCount = 0;
    GLfloat dTheta = 45.0f;
    GLfloat theta;
    std::vector<Vector3D*> thisCurvePnts = wvngCrv.myPnts;
    std::vector<Vector3D*> thisCurveTriads = wvngCrv.triadNorms;
    std::vector<Vector3D*> thisDervs = wvngCrv.derivatives;

    for(std::vector<Vector3D*>::iterator it = thisCurvePnts.begin(); it != thisCurvePnts.end(); it++,triadIndex++)
    {
        Vector3D norm = *thisCurveTriads[triadIndex];
        Vector3D derivative = *thisDervs[triadIndex];
        norm.normalize();
        derivative.normalize();
        Vector3D binorm = cross(norm,derivative);
        binorm.normalize();

        i = it-thisCurvePnts.begin();
        Vector3D tmp = *(*it);
        theta = 0.0f;
        for(int v=0; v<vNumPnts; v++)
        {
            Vector3D newPnt = tmp + THREAD_HRT*cos(theta*PI/180)*norm + THREAD_VERT*sin(theta*PI/180)*binorm;
            int index = i*vNumPnts*DIMENSION + v*DIMENSION;
            ctlpoints[index+0] = ( fabs(newPnt[0]) < 1.0e-5 ? 0.0f : newPnt[0] );
            ctlpoints[index+1] = ( fabs(newPnt[1]) < 1.0e-5 ? 0.0f : newPnt[1] );
            ctlpoints[index+2] = ( fabs(newPnt[2]) < 1.0e-5 ? 0.0f : newPnt[2] );
            theta += dTheta;
        }
    }

    firstRunCount = thisCurvePnts.size()-1;
    triadIndex = 1;
    for(int pnt=1; pnt<3; pnt++,triadIndex++)
    {
        Vector3D norm = *thisCurveTriads[triadIndex];
        Vector3D derivative = *thisDervs[triadIndex];
        norm.normalize();
        derivative.normalize();
        Vector3D binorm = cross(norm,derivative);
        binorm.normalize();

        i = firstRunCount + pnt;
        Vector3D tmp = *(thisCurvePnts[pnt]);
        theta = 0.0f;
        for(int v=0; v<vNumPnts; v++)
        {
            Vector3D newPnt = tmp + THREAD_HRT*cos(theta*PI/180)*norm + THREAD_VERT*sin(theta*PI/180)*binorm;
            int index = i*vNumPnts*DIMENSION + v*DIMENSION;
            ctlpoints[index+0] = ( fabs(newPnt[0]) < 1.0e-5 ? 0.0f : newPnt[0] );
            ctlpoints[index+1] = ( fabs(newPnt[1]) < 1.0e-5 ? 0.0f : newPnt[1] );
            ctlpoints[index+2] = ( fabs(newPnt[2]) < 1.0e-5 ? 0.0f : newPnt[2] );
            theta += dTheta;
        }
    }
}

void Mesh::generateWngTubes()
{
    for (std::vector<wngCrvCtrlPnts>::iterator crv = wngCrvs.begin(); crv != wngCrvs.end(); ++crv)
    {
        WvngTube *currTube = new WvngTube;
        wngCrvCtrlPnts currCurve = (*crv);
        currTube->uOrder = DEGREE+1;
        currTube->vOrder = DEGREE+1;
        currTube->uCtrlNum = currCurve.myPnts.size() + EXTRA_VERTICES;
        currTube->vCtrlNum = vNumPnts;
        currTube->uStride = currTube->vCtrlNum*DIMENSION;
        currTube->vStride = DIMENSION;
        currTube->uKnotCount = currTube->uCtrlNum + currTube->uOrder;
        currTube->vKnotCount = currTube->vCtrlNum + currTube->vOrder;
        currTube->uKnots = new GLfloat[currTube->uKnotCount];
        currTube->vKnots = new GLfloat[currTube->vKnotCount];
        generateKnotVectors(currTube->uKnots,currTube->uKnotCount,currTube->vKnots,currTube->vKnotCount);
        currTube->ctlPoints = new float[currTube->uCtrlNum*currTube->vCtrlNum*DIMENSION];
        generateTubeCtrlPnts(currCurve,currTube->ctlPoints);
        app::console()<<std::endl;
        mWvngTubes.push_back(currTube);
    }
}

void Mesh::regenerateThreads(bool isThicknessModified, CURVE_TYPE curveType)
{
    mWvngTubes.clear();
    baseCrvType = curveType;
    if(isThicknessModified) {
        THREAD_HRT	= 0.65*(TILE_THICKNESS/2.0);
        tile_map_.clear();
        wngCrvs.clear();
        pumpMesh();
        traverseFaces_PLUSES();
        //traverseFaces_TWISTS();
        if( baseCrvType == CATMULLROMSPLINE ) generateCtrlPnts4InterpolationSpline();
    }
    generateWngTubes();
}
