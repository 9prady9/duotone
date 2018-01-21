#include "cinder\params\Params.h"
#include "cinder/app/AppBasic.h"
#include "cinder\gl\Texture.h"
#include "cinder\BSpline.h"
#include "cinder\ImageIo.h"
#include "cinder\Vector.h"
#include "cinder/gl/gl.h"
#include "Resources.h"
#include "Weave.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const float white[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

class WeaveBoxApp : public AppBasic {
    public:
        void prepareSettings( Settings *settings );
        void setup();
        void resize();
        void mouseDown( MouseEvent event );
        void mouseDrag( MouseEvent event );
        void mouseWheel( MouseEvent event );
        void keyDown( KeyEvent event );
        void draw();

    private:
        void drawScene();
        void drawCube();
        void drawPlane();
        void updateThreadWidth();
        void updateThreadThickness();
        void updateTubesBaseCurve();

        GLfloat translateZ;
        GLfloat translateX;
        GLfloat translateY;
        GLfloat	rotationX;
        GLfloat	rotationY;
        GLfloat	rotationZ;
        GLfloat	prevZ;
        Vec2i	mLastPos;
        bool	wireframe;
        bool	defaultMesh;
        bool	baseMesh;
        bool	paintedMesh;
        bool	tiledMesh;
        bool	weavings;
        bool	lighting;
        int		combination;
        gl::Texture deflt;
        gl::Texture _0aTex, _0bTex;
        gl::Texture _0Tex, _1Tex, _2Tex;
        GLfloat	light_pos[4];
        GLfloat light_ambient[4];
        GLfloat light_diffuse[4];
        GLfloat light_specular[4];
        Mesh	*polygMesh;
        vector<string> mExtns;
        vector<string> texOptions;
        vector<string> curveType;
        CURVE_TYPE curvTyp;
        map<string,TILE_TEXTURES> texOpt;
        params::InterfaceGl mTwBar;
};

void WeaveBoxApp::prepareSettings( Settings *settings )
{
    translateX		= 0.0f;
    translateY		= 0.0f;
    translateZ		= -15.0f;
    rotationX		= 20.0f;
    rotationY		= 25.0f;
    rotationZ		= 0.0f;
    wireframe		= false;
    defaultMesh		= true;
    baseMesh		= true;
    paintedMesh		= true;
    tiledMesh		= false;
    weavings		= false;
    lighting		= true;
    combination		= 1;
    polygMesh = new Mesh;
    settings->setWindowSize(640,480);
    settings->setFrameRate(45.0f);
    texOptions.push_back("Blue Spots");
    texOptions.push_back("Yellow Spots");
    texOpt["Blue Spots"] = OA;
    texOpt["Yellow Spots"] = OB;
    mExtns.push_back("*.obj");
    mExtns.push_back("*.*");
    curvTyp = BSPLINE;
    curveType.push_back("B-Spline");
    curveType.push_back("Catmull-Rom Spline");

    mTwBar = params::InterfaceGl("Help & Params",Vec2i(275,500));
    mTwBar.addText( "H1", "label=`Hit 'l/L' key to load a mesh`");
    mTwBar.addText( "H2", "label=`Default Mode: Texture paint`");
    mTwBar.addSeparator();
    mTwBar.addText( "Tip0", "label=`Visibility Parameters(Hot Key):`");
    mTwBar.addParam("Wire Frame Mode   : ",&wireframe);
    mTwBar.addParam("Base Mesh  (b/B)  : ",&baseMesh);
    mTwBar.addParam("Painted Tex(p/P)  : ",&paintedMesh);
    mTwBar.addParam("Tiles      (t/T)  : ",&tiledMesh);
    mTwBar.addParam("Weaving    (v/V)  : ",&weavings);
    mTwBar.addSeparator();
    mTwBar.addSeparator();
    mTwBar.addText( "Tip5", "label=`Options for Duotone Surfaces only`" );
    mTwBar.addText( "Tip9", "label=`Strict key hit order: (4)->(5)->(6)`" );
    mTwBar.addText( "Tip6", "label=`Hit 4 to produce disconnected trees`" );
    mTwBar.addText( "Tip7", "label=`Hit 5 to join disconnected trees`" );
    mTwBar.addText( "Tip8", "label=`Hit 6 to join isolated vertices`" );
    mTwBar.addText( "Tip10", "label=`Final hit(6) will give a duotone surface`" );
    mTwBar.addSeparator();
    mTwBar.addSeparator();
    mTwBar.addText( "Tip11", "label=`Options for Weavings only`" );
    mTwBar.addParam("thread thickness ",&polygMesh->TILE_THICKNESS,"min=0.05 max=5.0 step=0.005");
    mTwBar.addButton("Apply thickness ",std::bind(&WeaveBoxApp::updateThreadThickness,this));
    mTwBar.addSeparator();
    mTwBar.addParam("thread width ",&polygMesh->THREAD_VERT,"min=0.01 max=5.0 step=0.005");
    mTwBar.addButton("Apply width ",std::bind(&WeaveBoxApp::updateThreadWidth,this));
    mTwBar.addSeparator();
    mTwBar.addParam("Base curve", curveType, (int*)&curvTyp);
    mTwBar.addButton("Click me to change base curve",std::bind(&WeaveBoxApp::updateTubesBaseCurve,this));
}

void WeaveBoxApp::setup()
{
    light_pos[0]		= 4.0; light_pos[1]		= 4.0; light_pos[2]		= 4.0; light_pos[3]		= 0.0;
    light_ambient[0]	= 1.0; light_ambient[1]	= 1.0; light_ambient[2] = 1.0; light_ambient[3] = 1.0;
    light_diffuse[0]	= 1.0; light_diffuse[1] = 1.0; light_diffuse[2] = 1.0; light_diffuse[3] = 1.0;
    light_specular[0]	= 1.0; light_specular[1]= 1.0; light_specular[2]= 1.0; light_specular[3]= 1.0;

    glLightfv( GL_LIGHT0, GL_AMBIENT, light_ambient );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, light_diffuse );
    glLightfv( GL_LIGHT0, GL_SPECULAR, light_specular );
    glLightfv( GL_LIGHT0, GL_POSITION, light_pos );
    glMaterialfv( GL_FRONT, GL_SPECULAR, white );
    glMaterialf( GL_FRONT, GL_SHININESS, 100.0f );
    deflt	= gl::Texture( loadImage ( loadResource( TUX_TEX ) ) );
    _0aTex	= gl::Texture( loadImage( loadResource( YELLOW_GLASS_TEX ) ) );
    _0bTex	= gl::Texture( loadImage( loadResource( BLUE_GLASS_TEX ) ) );
    _0Tex	= gl::Texture( loadImage( loadResource( TOUCH_TEX ) ) );
    _1Tex	= gl::Texture( loadImage( loadResource( CROSS_TEX ) ) );
    _2Tex	= gl::Texture( loadImage( loadResource( DOUBLE_CROSS_TEX ) ) );

    glClearColor( 0.25f, 0.25f, 0.25f, 1.0f );
    glShadeModel( GL_SMOOTH );
    glEnable( GL_NORMALIZE );
    glEnable( GL_AUTO_NORMAL );
    glEnable( GL_LIGHT0 );
    glEnable( GL_LIGHTING );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_TEXTURE_2D );
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
}

void WeaveBoxApp::mouseDown( MouseEvent event )
{
    mLastPos	= event.getPos();
}

void WeaveBoxApp::mouseDrag( MouseEvent event )
{
    Vec2i dPos	= event.getPos();
    dPos		= dPos - mLastPos;
    GLfloat dx	= (GLfloat)dPos.x / getWindowWidth();
    GLfloat dy	= (GLfloat)dPos.y / getWindowHeight();
    if( event.isShiftDown() && event.isLeftDown() )
    {
        translateX += 3*dx;
        translateY -= 3*dy;
    } else if( event.isLeftDown() )
    {
        rotationX += 180 * dy;
        rotationY += 180 * dx;
    }
    mLastPos = event.getPos();
    update();
}

void WeaveBoxApp::mouseWheel( MouseEvent event )
{
    double numSteps = event.getWheelIncrement()/5.0f;
    prevZ = translateZ;
    translateZ *= (GLfloat)pow(1.125,numSteps);
    update();
}

void WeaveBoxApp::keyDown( KeyEvent event )
{
    if( event.getCode() == KeyEvent::KEY_ESCAPE ) {
        quit();
    }else if( event.getChar() == 'w' || event.getChar() == 'W' ) {
        polygMesh->writeWavefrontObj();
    }else if( event.isAltDown() && event.getChar()=='l' ) {
        if( lighting ) {
            glDisable(GL_LIGHTING);
            lighting = false;
        } else {
            glEnable(GL_LIGHTING);
            lighting = true;
        }
    }else if( event.getChar() == 'l' || event.getChar()=='L' ) {
        try {
            fs::path p = getOpenFilePath( "", mExtns );
            if( ! p.empty() )
            {
                polygMesh->Clear();
                polygMesh->LoadObjFile(p.string().c_str());
                defaultMesh = false;
                stringstream sstream; sstream << polygMesh->getNumWvngCrvs();
                mTwBar.addSeparator();
                mTwBar.addText( "CycleCount", "label=`# of extracted cycles: " + sstream.str() + "`" );
                sstream.str(std::string()); sstream.clear();
                if(paintedMesh) {
                    polygMesh->resetTexturePaintMarkers();
                    polygMesh->paintTexturesOnMesh();
                }
            } else {
                defaultMesh = true;
            }
        }
        catch( ... ) {
            console() << "Unable to load the object file." << std::endl;
        }
    }else if( event.getChar() == 'b' || event.getChar()=='B' ) {
        baseMesh = !baseMesh;
    }else if( event.isAltDown() && event.getChar() == 'p' ) {
        if(paintedMesh) {
            polygMesh->resetTexturePaintMarkers();
            polygMesh->paintTexturesOnMesh();
        }
    }else if( event.getChar() == 'p' ) {
        if(!paintedMesh)
            polygMesh->paintTexturesOnMesh();
        paintedMesh = !paintedMesh;
    }else if( event.getChar() == 't' || event.getChar()=='T' ) {
        tiledMesh = !tiledMesh;
    }else if( event.getChar() == 'v' || event.getChar()=='V' ) {
        weavings = !weavings;
    }else if( event.getChar() == 'h' || event.getChar()=='H' ) {
        mTwBar.hide();
    }else if( event.getChar() == 's' || event.getChar()=='S' ) {
        mTwBar.show();
    }else if( event.getChar() == '9' ) {
        polygMesh->resetToTwistMode();
    }else if( event.getChar() == '0' ) {
        polygMesh->resetToPlusMode();
    }else if( event.getChar() == '1' ) {
        combination = 1;
    }else if( event.getChar() == '2' ) {
        combination = 2;
    }else if( event.getChar() == '3' ) {
        combination = 3;
    }else if( event.getChar() == '4' ) {
        polygMesh->connectBlobsPhase1();
    }else if( event.getChar() == '5' ) {
        polygMesh->connectBlobsPhase2();
    }else if( event.getChar() == '6' ) {
        polygMesh->connectBlobsPhase3();
    }else if( event.getChar() == '7' ) {
        polygMesh->connectBlobsPhase1();
        polygMesh->connectBlobsPhase2();
        polygMesh->connectBlobsPhase3();
    }
    update();
}

void WeaveBoxApp::resize()
{
    glViewport( 0, 0, getWindowWidth(), getWindowHeight() );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 45.0f, GLfloat(getWindowWidth()) / getWindowHeight(), 1.0f, 100.0f );
    glMatrixMode( GL_MODELVIEW );
}

void WeaveBoxApp::draw()
{
    glClearColor( 0.25f, 0.25f, 0.25f, 1.0f );
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity();
    glTranslatef( translateX, translateY, translateZ );
    glRotatef( rotationX, 1.0f, 0.0f, 0.0f );
    glRotatef( rotationY, 0.0f, 1.0f, 0.0f );
    glRotatef( rotationZ, 0.0f, 0.0f, 1.0f );

    if( lighting && fabs( prevZ-translateZ )<1.0e-6 )
    {
        glDisable( GL_LIGHTING );
        light_pos[0] = 4.0f*translateZ; light_pos[1] = 4.0f*translateZ;
        light_pos[2] = 4.0f*translateZ; light_pos[3] = 0.0f*translateZ;
        glLightfv( GL_LIGHT0, GL_POSITION, light_pos );
        glEnable( GL_LIGHTING );
    }

    if(defaultMesh)
    {
        deflt.bind();
        drawCube();
        glPushMatrix();
        glScalef( 8.0f, 1.5f, 8.0f );
        glTranslatef( 0.0f, -1.0f, 0.0f );
        drawPlane();
        glPopMatrix();
        deflt.unbind();
    }
    else
    {
        switch(combination)
        {
            case 1:
                polygMesh->drawMesh( wireframe, lighting, baseMesh, paintedMesh, tiledMesh, weavings, _0aTex.getId(), _0bTex.getId() );
                break;
            case 2:
                polygMesh->drawMesh( wireframe, lighting, baseMesh, paintedMesh, tiledMesh, weavings, _0Tex.getId(), _1Tex.getId() );
                break;
            case 3:
                polygMesh->drawMesh( wireframe, lighting, baseMesh, paintedMesh, tiledMesh, weavings, _0Tex.getId(), _0Tex.getId() );
                break;
            case 4:
                polygMesh->drawMesh( wireframe, lighting, baseMesh, paintedMesh, tiledMesh, weavings, _2Tex.getId(), _2Tex.getId() );
                break;
            default:
                break;
        }
    }
    mTwBar.draw();
}

void WeaveBoxApp::drawCube()
{
    glBegin(GL_QUADS);			// Start Drawing Quads
    // Front Face
    glNormal3f( 0.0f, 0.0f, 1.0f);		// Normal Facing Forward
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    // Back Face
    glNormal3f( 0.0f, 0.0f,-1.0f);		// Normal Facing Away
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
    // Top Face
    glNormal3f( 0.0f, 1.0f, 0.0f);		// Normal Facing Up
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    // Bottom Face
    glNormal3f( 0.0f,-1.0f, 0.0f);		// Normal Facing Down
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    // Right face
    glNormal3f( 1.0f, 0.0f, 0.0f);		// Normal Facing Right
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f, -1.0f, -1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  1.0f, -1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f( 1.0f,  1.0f,  1.0f);	// Top Left Of The Texture and Quad
    glTexCoord2f(0.0f, 1.0f); glVertex3f( 1.0f, -1.0f,  1.0f);	// Bottom Left Of The Texture and Quad
    // Left Face
    glNormal3f(-1.0f, 0.0f, 0.0f);		// Normal Facing Left
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f, -1.0f, -1.0f);	// Bottom Left Of The Texture and Quad
    glTexCoord2f(1.0f, 1.0f); glVertex3f(-1.0f, -1.0f,  1.0f);	// Bottom Right Of The Texture and Quad
    glTexCoord2f(1.0f, 0.0f); glVertex3f(-1.0f,  1.0f,  1.0f);	// Top Right Of The Texture and Quad
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  1.0f, -1.0f);	// Top Left Of The Texture and Quad
    glEnd();					// Done Drawing Quads
}

void WeaveBoxApp::drawPlane()
{
    glBegin(GL_QUADS);
    glNormal3f( 0.0f, 1.0f, 0.0f);
    glTexCoord2f(0.0f, 0.0f); glVertex3f(-1.0f,  0.0f, -1.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex3f(-1.0f,  0.0f,  1.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex3f( 1.0f,  0.0f,  1.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex3f( 1.0f,  0.0f, -1.0f);
    glEnd();
}

void WeaveBoxApp::updateThreadWidth()
{
    polygMesh->regenerateThreads(false);
}

void WeaveBoxApp::updateThreadThickness()
{
    polygMesh->regenerateThreads(true);
}

void WeaveBoxApp::updateTubesBaseCurve()
{
    polygMesh->regenerateThreads(true,curvTyp);
}

CINDER_APP_BASIC( WeaveBoxApp, RendererGl )