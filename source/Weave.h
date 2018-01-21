#ifndef WEAVE_H
#define WEAVE_H

#include "renderable_object.h"
#include "cinder\app\AppBasic.h"
#include "cinder\gl\Texture.h"
#include "cinder\ImageIo.h"
#include "cinder\BSpline.h"
#include "cinder\Vector.h"
#include "cinder/gl/gl.h"
#include "Resources.h"
#include "circularlist.h"
#include "gl\GLU.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>

class Vertex;
class Edge;
class Face;
class Vector3D;

#define PI 3.1415926535897
#define DEGREE 3
#define DIMENSION 3
#define EXTRA_VERTICES 2
#define FACE_NUM_V 4
#define NUM_SEGMENTS 512

enum KNOT { NOTHING, PLUS1, PLUS2, TWIST1, TWIST2 };
enum TILE_TEXTURES { OA, OB, TOUCH, CROSS, DOUBLE_CROSS };
enum USAGE { YES, NO };
enum COLORS { SOLID1, SOLID2 };
enum CURVE_TYPE { BSPLINE, CATMULLROMSPLINE };

typedef std::vector<Vector3D*>::iterator Vec3DPtrIter;

class Colour {
    public:
        Colour() { r=g=b=0; a=1; }
        Colour(float red, float green, float blue, float alpha=1) {
            r = red;
            g = green;
            b = blue;
            a = alpha;
        }
        float r,g,b,a;
};

class Tile
{
    public:
        Tile(unsigned int num_vertices);
        ~Tile();
        Vector3D trilinearInterpolation(float u,float v,float t);
        void addUpAndDown(Vertex *pnt, Vector3D *u, Vector3D *d) {
            actVerts.push_back(pnt);
            up.push_back(u);
            down.push_back(d);
        }
        void generateCurvePoints();
        void generateTwist1Points();
        void generateTwist2Points();
        void swapCurveMidPoints();
        void findCurveAndDirection(Vertex* end, Vertex* othEnd, bool &isCurve1, bool &isRev);

        /* Attirbutes */
        std::vector<Vector3D*> up;
        std::vector<Vector3D*> down;
        std::vector<Vertex*> actVerts;
        std::map<Vertex*,DCircularList*> texCoords;
        int marked;			// 0 is base level - texture pasted; no replaces done; indicates number of replaces if > 0
        TILE_TEXTURES tex;

        unsigned int n;
        std::vector<Vector3D*> curve1;
        std::vector<Vector3D*> curve2;
        KNOT knotType;
        /* Visited count should not be more
         * than 2 since two curves on each face */
        unsigned int visited;
        USAGE isCrv1Used;
        USAGE isCrv2Used;
        USAGE isUsedCrv1Rev;
        USAGE isUsedCrv2Rev;
};

class wngCrvCtrlPnts {
    public:
        std::vector<Vector3D*> myPnts;
        std::vector<Vector3D*> derivatives;
        std::vector<Vector3D*> triadNorms;

        ~wngCrvCtrlPnts() {
            myPnts.clear();
            triadNorms.clear();
            derivatives.clear();
        }
};

class WvngTube{
    public:
        GLint uOrder;
        GLint uKnotCount;
        GLfloat* uKnots;
        GLint uStride;
        GLint uCtrlNum;
        GLint vOrder;
        GLint vKnotCount;
        GLfloat* vKnots;
        GLint vStride;
        GLint vCtrlNum;
        GLfloat* ctlPoints;

        ~WvngTube() {
            delete[] uKnots;
            delete[] vKnots;
            delete[] ctlPoints;
        }
};

class Mesh : public RenderableObject {
    public:
        Mesh() :vNumPnts(12) {
            numFacesObjFile_ = 0;
            theNurb = gluNewNurbsRenderer();
            gluNurbsProperty( theNurb, GLU_SAMPLING_TOLERANCE, 25.0f );
            TILE_THICKNESS	= 0.1f;
            THREAD_HRT	= 0.65*(TILE_THICKNESS/2.0);
            THREAD_VERT	= 0.05f;
            _0aHead	= 0;
            _0bHead	= 0;
            baseCrvType = BSPLINE;
            prepareTextureSets();
            createTileTexCoordList();
        }
        ~Mesh() {
            Clear();
            texture_Bcorner.clear();
            texture_Ycorner.clear();
            compat_Bcorner.clear();
            compat_Ycorner.clear();
            delete _0aHead;
            delete _0bHead;
        }
        bool LoadObjFile(const char* objFileName);
        int  getNumEdges() { return edges_.size(); }
        int  getNumFaces() { return faces_.size(); }
        int  getNumWvngCrvs() { return wngCrvs.size(); }
        void resetToTwistMode() {}
        void resetToPlusMode() {}
        bool Clear();
        void drawMesh(bool wireframe, bool lighting, bool drawBaseMesh, bool paintedMesh, bool tiledMesh, bool drawWeaving,
                GLint TURN1_OR_TOUCH, GLint TURN2_OR_CROSS, GLint DOUBLE_CROSS=0);
        void pumpMesh();
        void traverseFaces_PLUSES();
        void traverseFaces_TWISTS();

        void resetTexturePaintMarkers();
        void paintTexturesOnMesh();
        bool findUnConnectedFace(Face **returnValue, Vertex **returnVertex);
        void incMarkerOnFacesIncident(Vertex *v);
        Vertex* getNextPivotInFace(Face *f, Vertex *v, char &colorName, int &index1, int &index2);
        void connectBlobsPhase1();
        void connectBlobsPhase2();
        void connectBlobsPhase3();
        void writeWavefrontObj();

        void generateCtrlPnts4InterpolationSpline();
        void generateKnotVectors(GLfloat *uKnots, GLint uKnotCount, GLfloat *vKnots, GLint vKnotCount);
        void generateTubeCtrlPnts(const wngCrvCtrlPnts &wvngCrv, float *);
        void generateWngTubes();
        void regenerateThreads(bool isThicknessModified, CURVE_TYPE curveType=BSPLINE);

        // Attributes
        float TILE_THICKNESS;
        float THREAD_HRT;
        float THREAD_VERT;
        CURVE_TYPE baseCrvType;
        DCircularList *_0aHead;
        DCircularList *_0bHead;

    private:
        std::string getOriginalFileName(std::string filename,std::string &path);
        void cacheFaceNormals();
        void calculateVertexNormals();
        void prepareTextureSets();
        void createTileTexCoordList();
        bool anyUnvisitedFaces(Face **retFacePntr,Tile **correspondingTile);

        int numFacesObjFile_;
        std::string fileName_;
        std::map<Vertex*, Vector3D*> vNormals_;
        std::map<Face*, Vector3D*> fNormals_;
        std::map<Face*,Tile*> tile_map_;

        std::vector<wngCrvCtrlPnts> wngCrvs;
        std::vector<WvngTube*> mWvngTubes;
        GLUnurbsObj *theNurb;
        const GLint vNumPnts;

        std::vector<char> texture_Bcorner;
        std::vector<char> texture_Ycorner;
        std::vector<char> compat_Bcorner;
        std::vector<char> compat_Ycorner;
        std::map<Vertex*,int> vertex_treeIDMap_;
        std::map<int, std::set<Vertex*>> treeID_VertexSetMap_;
};

#endif
